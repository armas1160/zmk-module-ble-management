/**
 * BLE Management Feature - Custom Studio RPC Handler
 *
 * This file implements BLE management functionality for ZMK Studio.
 * It provides APIs to:
 * - View and manage BLE profiles
 * - Set custom names for profiles (tied to BLE address)
 * - Switch active profiles
 * - Unpair profiles
 * - Manage split keyboard connections
 * - Set and get output priority (USB or BLE)
 */

#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zmk/ble.h>
#include <zmk/ble_management/ble_management.pb.h>
#include <zmk/endpoints.h>
#include <zmk/studio/custom.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_PERIPHERAL)
#include <zmk/split/bluetooth/peripheral.h>
#endif
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Structure to store profile name tied to BLE address
struct profile_name_entry {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    bt_addr_le_t addr;
#endif
    char name[32];
};

// Profile names cache (in memory)
#if IS_ENABLED(CONFIG_ZMK_BLE)
static struct profile_name_entry profile_names[ZMK_BLE_PROFILE_COUNT];
#else
static struct profile_name_entry
    profile_names[1];  // Dummy array when BLE is disabled
#endif

/**
 * Metadata for the custom subsystem.
 */
static struct zmk_rpc_custom_subsystem_meta ble_management_meta = {
    ZMK_RPC_CUSTOM_SUBSYSTEM_UI_URLS(
        "https://cormoran.github.io/zmk-module-ble-management/"),
    /* Unpairing, switching profiles and forgetting the split bond change the keyboard, so require a Studio unlock */
    .security = ZMK_STUDIO_RPC_HANDLER_SECURED,
};

/**
 * Register the custom RPC subsystem.
 */
ZMK_RPC_CUSTOM_SUBSYSTEM(cormoran_ble, &ble_management_meta,
                         ble_management_rpc_handle_request);

ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER(cormoran_ble,
                                         zmk_ble_management_Response);

// Forward declarations
static int handle_get_profiles_request(
    const zmk_ble_management_GetProfilesRequest *req,
    zmk_ble_management_Response *resp);
static int handle_set_profile_name_request(
    const zmk_ble_management_SetProfileNameRequest *req,
    zmk_ble_management_Response *resp);
static int handle_switch_profile_request(
    const zmk_ble_management_SwitchProfileRequest *req,
    zmk_ble_management_Response *resp);
static int handle_unpair_profile_request(
    const zmk_ble_management_UnpairProfileRequest *req,
    zmk_ble_management_Response *resp);
static int handle_get_split_info_request(
    const zmk_ble_management_GetSplitInfoRequest *req,
    zmk_ble_management_Response *resp);
static int handle_forget_split_bond_request(
    const zmk_ble_management_ForgetSplitBondRequest *req,
    zmk_ble_management_Response *resp);
static int handle_set_output_priority_request(
    const zmk_ble_management_SetOutputPriorityRequest *req,
    zmk_ble_management_Response *resp);
static int handle_get_output_priority_request(
    const zmk_ble_management_GetOutputPriorityRequest *req,
    zmk_ble_management_Response *resp);

#if IS_ENABLED(CONFIG_ZMK_BLE)
/**
 * Names are matched by the 48-bit address only. The settings key holds no
 * address type, so every name is read back as a public address after a
 * reboot, and a name saved for a random-address host would never match again.
 */
static bool same_addr(const bt_addr_le_t *a, const bt_addr_le_t *b) {
    return bt_addr_eq(&a->a, &b->a);
}

static void name_setting_key(const bt_addr_le_t *addr, char *key,
                             size_t len) {
    char addr_str[BT_ADDR_STR_LEN];
    bt_addr_to_str(&addr->a, addr_str, sizeof(addr_str));
    snprintf(key, len, "ble_mgmt/name/%s", addr_str);
}

static void delete_saved_name(const bt_addr_le_t *addr) {
    char key[64];
    name_setting_key(addr, key, sizeof(key));
    int err = settings_delete(key);
    if (err) {
        LOG_WRN("Failed to delete %s: %d", key, err);
    }
}

/**
 * Whether the host paired to some profile has this address. An open profile
 * (BT_ADDR_LE_ANY) has no host, so a name saved under that address is stale.
 */
static bool is_profile_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        bt_addr_le_t *profile_addr = zmk_ble_profile_address(i);
        if (!bt_addr_le_eq(profile_addr, BT_ADDR_LE_ANY) &&
            same_addr(profile_addr, addr)) {
            return true;
        }
    }
    return false;
}
#endif

/**
 * Get profile name from cache based on BLE address
 */
static const char *get_profile_name(const bt_addr_le_t *addr) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (!addr) {
        return "";
    }

    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (same_addr(&profile_names[i].addr, addr)) {
            return profile_names[i].name;
        }
    }
#endif
    return "";
}

/**
 * Save profile name to settings and cache
 */
static int save_profile_name(const bt_addr_le_t *addr, const char *name) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (!addr || !name) {
        return -EINVAL;
    }

    // Find existing entry or empty slot
    int slot = -1;
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (same_addr(&profile_names[i].addr, addr)) {
            slot = i;
            break;
        }
        if (slot == -1 &&
            bt_addr_le_eq(&profile_names[i].addr, BT_ADDR_LE_NONE)) {
            slot = i;
        }
    }

    if (slot == -1) {
        // Every slot is taken. Names of hosts unpaired some other way (e.g.
        // &bt BT_CLR) stay saved, so reuse the slot of a name whose host is
        // no longer paired to any profile, and delete that saved name.
        for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
            if (!is_profile_addr(&profile_names[i].addr)) {
                delete_saved_name(&profile_names[i].addr);
                slot = i;
                break;
            }
        }
    }

    if (slot == -1) {
        LOG_WRN("No slot available for profile name");
        return -ENOMEM;
    }

    // Update cache
    bt_addr_le_copy(&profile_names[slot].addr, addr);
    strncpy(profile_names[slot].name, name,
            sizeof(profile_names[slot].name) - 1);
    profile_names[slot].name[sizeof(profile_names[slot].name) - 1] = '\0';

    // Save to settings
    char setting_name[64];
    name_setting_key(addr, setting_name, sizeof(setting_name));

    return settings_save_one(setting_name, name, strlen(name) + 1);
#else
    return -ENOTSUP;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_BLE)
// A saved name did not fit in the cache while loading.
static bool name_dropped_on_load;

/**
 * Address from the part of a name key after "name/". The key holds no address
 * type, so it reads back as public; same_addr() ignores the type.
 */
static int parse_name_addr(const char *next, bt_addr_le_t *addr) {
    char addr_str[BT_ADDR_STR_LEN];
    strncpy(addr_str, next, sizeof(addr_str) - 1);
    addr_str[sizeof(addr_str) - 1] = '\0';
    return bt_addr_le_from_str(addr_str, "public", addr);
}
#endif

/**
 * Settings callback for loading profile names
 */
static int profile_names_settings_set(const char *name, size_t len,
                                      settings_read_cb read_cb, void *cb_arg) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    const char *next;
    int rc;

    if (settings_name_steq(name, "name", &next) && next) {
        bt_addr_le_t addr;
        if (parse_name_addr(next, &addr) != 0) {
            LOG_WRN("Failed to parse address: %s", next);
            return 0;
        }

        // Find or allocate slot
        int slot = -1;
        for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
            if (same_addr(&profile_names[i].addr, &addr)) {
                slot = i;
                break;
            }
            if (slot == -1 &&
                bt_addr_le_eq(&profile_names[i].addr, BT_ADDR_LE_NONE)) {
                slot = i;
            }
        }

        if (slot == -1) {
            LOG_WRN("No slot for loading profile name");
            name_dropped_on_load = true;
            return 0;
        }

        bt_addr_le_copy(&profile_names[slot].addr, &addr);
        rc = read_cb(cb_arg, profile_names[slot].name,
                     sizeof(profile_names[slot].name));
        if (rc >= 0) {
            profile_names[slot].name[sizeof(profile_names[slot].name) - 1] =
                '\0';
            LOG_DBG("Loaded profile name for %s: %s", next,
                    profile_names[slot].name);
        }
    }
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BLE)
struct stale_names {
    bt_addr_le_t addrs[8];
    size_t count;
};

/**
 * Called for every saved name by review_saved_names(). Caches the name of a
 * paired host if it is not cached yet, and collects the others for deletion.
 */
static int review_saved_name(const char *key, size_t len,
                             settings_read_cb read_cb, void *cb_arg,
                             void *param) {
    struct stale_names *stale = param;
    const char *next;
    bt_addr_le_t addr;

    if (!settings_name_steq(key, "name", &next) || !next ||
        parse_name_addr(next, &addr) != 0) {
        return 0;
    }
    if (is_profile_addr(&addr)) {
        return profile_names_settings_set(key, len, read_cb, cb_arg);
    }
    if (stale->count < ARRAY_SIZE(stale->addrs)) {
        bt_addr_le_copy(&stale->addrs[stale->count++], &addr);
    }
    return 0;
}

/**
 * Free the slots of hosts that are no longer paired, go through the saved
 * names again to cache the paired hosts' names, and delete the rest.
 */
static void review_saved_names(struct k_work *work) {
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (!is_profile_addr(&profile_names[i].addr)) {
            bt_addr_le_copy(&profile_names[i].addr, BT_ADDR_LE_NONE);
            profile_names[i].name[0] = '\0';
        }
    }

    // Delete once the storage has been read, not while it is being read
    struct stale_names stale = {.count = 0};
    settings_load_subtree_direct("ble_mgmt", review_saved_name, &stale);
    for (size_t i = 0; i < stale.count; i++) {
        delete_saved_name(&stale.addrs[i]);
    }
}

static K_WORK_DELAYABLE_DEFINE(review_saved_names_work, review_saved_names);
#endif

/**
 * Settings commit, run once everything is loaded (the BLE profiles too).
 * Firmware from before names were deleted on unpair could leave more saved
 * names than fit in the cache, and which ones load first is up to the
 * storage, so a paired host may have lost its name. If any name was dropped,
 * review the saved names. That reads the storage again and writes to it, so
 * it runs on the system work queue once the load is over, not here on the
 * thread running settings_load() (the main thread, with a small stack).
 */
static int profile_names_settings_commit(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (name_dropped_on_load) {
        name_dropped_on_load = false;
        k_work_schedule(&review_saved_names_work, K_SECONDS(1));
    }
#endif

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(ble_mgmt, "ble_mgmt", NULL,
                               profile_names_settings_set,
                               profile_names_settings_commit, NULL);

/**
 * Main request handler for the custom RPC subsystem.
 */
static bool ble_management_rpc_handle_request(
    const zmk_custom_CallRequest *raw_request, pb_callback_t *encode_response) {
    zmk_ble_management_Response *resp =
        ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER_ALLOCATE(cormoran_ble,
                                                          encode_response);

    zmk_ble_management_Request req = zmk_ble_management_Request_init_zero;

    // Decode the incoming request
    pb_istream_t req_stream = pb_istream_from_buffer(raw_request->payload.bytes,
                                                     raw_request->payload.size);
    if (!pb_decode(&req_stream, zmk_ble_management_Request_fields, &req)) {
        LOG_WRN("Failed to decode ble_management request: %s",
                PB_GET_ERROR(&req_stream));
        zmk_ble_management_ErrorResponse err =
            zmk_ble_management_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message), "Failed to decode request");
        resp->which_response_type = zmk_ble_management_Response_error_tag;
        resp->response_type.error = err;
        return true;
    }

    int rc = 0;
    switch (req.which_request_type) {
        case zmk_ble_management_Request_get_profiles_tag:
            rc = handle_get_profiles_request(&req.request_type.get_profiles,
                                             resp);
            break;
        case zmk_ble_management_Request_set_profile_name_tag:
            rc = handle_set_profile_name_request(
                &req.request_type.set_profile_name, resp);
            break;
        case zmk_ble_management_Request_switch_profile_tag:
            rc = handle_switch_profile_request(&req.request_type.switch_profile,
                                               resp);
            break;
        case zmk_ble_management_Request_unpair_profile_tag:
            rc = handle_unpair_profile_request(&req.request_type.unpair_profile,
                                               resp);
            break;
        case zmk_ble_management_Request_get_split_info_tag:
            rc = handle_get_split_info_request(&req.request_type.get_split_info,
                                               resp);
            break;
        case zmk_ble_management_Request_forget_split_bond_tag:
            rc = handle_forget_split_bond_request(
                &req.request_type.forget_split_bond, resp);
            break;
        case zmk_ble_management_Request_set_output_priority_tag:
            rc = handle_set_output_priority_request(
                &req.request_type.set_output_priority, resp);
            break;
        case zmk_ble_management_Request_get_output_priority_tag:
            rc = handle_get_output_priority_request(
                &req.request_type.get_output_priority, resp);
            break;
        default:
            LOG_WRN("Unsupported request type: %d", req.which_request_type);
            rc = -ENOTSUP;
    }

    if (rc != 0) {
        zmk_ble_management_ErrorResponse err =
            zmk_ble_management_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message),
                 "Failed to process request: %d", rc);
        resp->which_response_type = zmk_ble_management_Response_error_tag;
        resp->response_type.error = err;
    }
    return true;
}

/**
 * Handle GetProfilesRequest
 */
static int handle_get_profiles_request(
    const zmk_ble_management_GetProfilesRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("GetProfilesRequest");

    zmk_ble_management_GetProfilesResponse result =
        zmk_ble_management_GetProfilesResponse_init_zero;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    result.max_profiles = ZMK_BLE_PROFILE_COUNT;

    int active = zmk_ble_active_profile_index();

    for (uint8_t i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        zmk_ble_management_ProfileInfo *profile = &result.profiles[i];
        profile->index                          = i;
        profile->is_open                        = zmk_ble_profile_is_open(i);
        profile->is_connected = zmk_ble_profile_is_connected(i);
        profile->is_active    = (i == active);

        // Get BLE address
        bt_addr_le_t *addr = zmk_ble_profile_address(i);
        if (addr && !bt_addr_le_eq(addr, BT_ADDR_LE_NONE)) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            strncpy(profile->address, addr_str, sizeof(profile->address) - 1);
            profile->address[sizeof(profile->address) - 1] = '\0';

            // Get custom name
            const char *name = get_profile_name(addr);
            if (name && name[0] != '\0') {
                strncpy(profile->name, name, sizeof(profile->name) - 1);
                profile->name[sizeof(profile->name) - 1] = '\0';
            }
        }
    }

    result.profiles_count = ZMK_BLE_PROFILE_COUNT;
#else
    result.max_profiles   = 0;
    result.profiles_count = 0;
#endif

    resp->which_response_type = zmk_ble_management_Response_get_profiles_tag;
    resp->response_type.get_profiles = result;
    return 0;
}

/**
 * Handle SetProfileNameRequest
 */
static int handle_set_profile_name_request(
    const zmk_ble_management_SetProfileNameRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("SetProfileNameRequest: index=%d, name=%s", req->index, req->name);

    zmk_ble_management_SetProfileNameResponse result =
        zmk_ble_management_SetProfileNameResponse_init_zero;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (req->index >= ZMK_BLE_PROFILE_COUNT) {
        LOG_WRN("Invalid profile index: %d", req->index);
        result.success = false;
    } else {
        // An open profile (BT_ADDR_LE_ANY) has no host to name. A name saved
        // under that address would show on every open profile.
        bt_addr_le_t *addr = zmk_ble_profile_address(req->index);
        if (addr && !bt_addr_le_eq(addr, BT_ADDR_LE_NONE) &&
            !bt_addr_le_eq(addr, BT_ADDR_LE_ANY)) {
            int rc         = save_profile_name(addr, req->name);
            result.success = (rc == 0);
        } else {
            LOG_WRN("Profile %d has no address", req->index);
            result.success = false;
        }
    }
#else
    result.success = false;
#endif

    resp->which_response_type =
        zmk_ble_management_Response_set_profile_name_tag;
    resp->response_type.set_profile_name = result;
    return 0;
}

/**
 * Handle SwitchProfileRequest
 */
static int handle_switch_profile_request(
    const zmk_ble_management_SwitchProfileRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("SwitchProfileRequest: index=%d", req->index);

    zmk_ble_management_SwitchProfileResponse result =
        zmk_ble_management_SwitchProfileResponse_init_zero;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (req->index >= ZMK_BLE_PROFILE_COUNT) {
        LOG_WRN("Invalid profile index: %d", req->index);
        result.success = false;
    } else {
        int rc         = zmk_ble_prof_select(req->index);
        result.success = (rc == 0);
    }
#else
    result.success = false;
#endif

    resp->which_response_type = zmk_ble_management_Response_switch_profile_tag;
    resp->response_type.switch_profile = result;
    return 0;
}

/**
 * Handle UnpairProfileRequest
 */
static int handle_unpair_profile_request(
    const zmk_ble_management_UnpairProfileRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("UnpairProfileRequest: index=%d", req->index);

    zmk_ble_management_UnpairProfileResponse result =
        zmk_ble_management_UnpairProfileResponse_init_zero;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    if (req->index >= ZMK_BLE_PROFILE_COUNT) {
        LOG_WRN("Invalid profile index: %d", req->index);
        result.success = false;
    } else {
        // Clear the profile name from the cache and from settings. Left in
        // settings, it is loaded again on the next boot and keeps a slot.
        bt_addr_le_t *addr = zmk_ble_profile_address(req->index);
        if (addr && !bt_addr_le_eq(addr, BT_ADDR_LE_NONE) &&
            !bt_addr_le_eq(addr, BT_ADDR_LE_ANY)) {
            for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
                if (same_addr(&profile_names[i].addr, addr)) {
                    bt_addr_le_copy(&profile_names[i].addr, BT_ADDR_LE_NONE);
                    profile_names[i].name[0] = '\0';
                    break;
                }
            }
            delete_saved_name(addr);
        }
        int active = zmk_ble_active_profile_index();
        int rc     = zmk_ble_prof_select(req->index);
        if (rc == 0) {
            zmk_ble_clear_bonds();
            if (active != req->index) {
                rc = zmk_ble_prof_select(active);
            }
        }
        result.success = (rc == 0);
    }
#else
    result.success = false;
#endif

    resp->which_response_type = zmk_ble_management_Response_unpair_profile_tag;
    resp->response_type.unpair_profile = result;
    return 0;
}

/**
 * Handle GetSplitInfoRequest
 */
static int handle_get_split_info_request(
    const zmk_ble_management_GetSplitInfoRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("GetSplitInfoRequest");

    zmk_ble_management_GetSplitInfoResponse result =
        zmk_ble_management_GetSplitInfoResponse_init_zero;
    zmk_ble_management_SplitInfo *info = &result.info;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    info->is_split = true;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    info->is_central = true;
    // For central, check if peripheral is connected
    // Note: ZMK doesn't expose a direct API for this, so we'll set it to false
    // for now
    info->peripheral_connected = false;
    info->central_bonded       = false;
#elif IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_PERIPHERAL)
    info->is_central           = false;
    info->peripheral_connected = false;
    info->central_bonded       = zmk_split_bt_peripheral_is_bonded();
#endif
#else
    info->is_split             = false;
    info->is_central           = false;
    info->peripheral_connected = false;
    info->central_bonded       = false;
#endif

    resp->which_response_type = zmk_ble_management_Response_get_split_info_tag;
    resp->response_type.get_split_info = result;
    return 0;
}

/**
 * Handle ForgetSplitBondRequest
 */
static int handle_forget_split_bond_request(
    const zmk_ble_management_ForgetSplitBondRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("ForgetSplitBondRequest");

    zmk_ble_management_ForgetSplitBondResponse result =
        zmk_ble_management_ForgetSplitBondResponse_init_zero;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE)
    // Clear all bonds to reset split connection
    zmk_ble_clear_all_bonds();
    result.success = true;
#else
    LOG_WRN("Split BLE not enabled");
    result.success = false;
#endif

    resp->which_response_type =
        zmk_ble_management_Response_forget_split_bond_tag;
    resp->response_type.forget_split_bond = result;
    return 0;
}

/**
 * Handle SetOutputPriorityRequest
 */
static int handle_set_output_priority_request(
    const zmk_ble_management_SetOutputPriorityRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("SetOutputPriorityRequest: priority=%d", req->priority);

    zmk_ble_management_SetOutputPriorityResponse result =
        zmk_ble_management_SetOutputPriorityResponse_init_zero;

    // Convert protobuf enum to ZMK transport enum
    enum zmk_transport transport;
    switch (req->priority) {
        case zmk_ble_management_OutputPriority_OUTPUT_PRIORITY_USB:
            transport = ZMK_TRANSPORT_USB;
            break;
        case zmk_ble_management_OutputPriority_OUTPUT_PRIORITY_BLE:
            transport = ZMK_TRANSPORT_BLE;
            break;
        default:
            LOG_WRN("Invalid output priority: %d", req->priority);
            result.success = false;
            resp->which_response_type =
                zmk_ble_management_Response_set_output_priority_tag;
            resp->response_type.set_output_priority = result;
            return 0;
    }

    int rc         = zmk_endpoint_set_preferred_transport(transport);
    result.success = (rc == 0);

    resp->which_response_type =
        zmk_ble_management_Response_set_output_priority_tag;
    resp->response_type.set_output_priority = result;
    return 0;
}

/**
 * Handle GetOutputPriorityRequest
 */
static int handle_get_output_priority_request(
    const zmk_ble_management_GetOutputPriorityRequest *req,
    zmk_ble_management_Response *resp) {
    LOG_DBG("GetOutputPriorityRequest");

    zmk_ble_management_GetOutputPriorityResponse result =
        zmk_ble_management_GetOutputPriorityResponse_init_zero;

    // Get the currently selected endpoint
    enum zmk_transport current = zmk_endpoint_get_preferred_transport();

    // Convert ZMK transport enum to protobuf enum
    switch (current) {
        case ZMK_TRANSPORT_USB:
            result.priority =
                zmk_ble_management_OutputPriority_OUTPUT_PRIORITY_USB;
            break;
        case ZMK_TRANSPORT_BLE:
            result.priority =
                zmk_ble_management_OutputPriority_OUTPUT_PRIORITY_BLE;
            break;
        default:
            LOG_WRN("Unknown transport type: %d", current);
            result.priority =
                zmk_ble_management_OutputPriority_OUTPUT_PRIORITY_BLE;
            break;
    }

    resp->which_response_type =
        zmk_ble_management_Response_get_output_priority_tag;
    resp->response_type.get_output_priority = result;
    return 0;
}

/**
 * Initialize profile names on boot
 */
static int profile_names_init(void) {
#if IS_ENABLED(CONFIG_ZMK_BLE)
    // Initialize all entries to empty
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        bt_addr_le_copy(&profile_names[i].addr, BT_ADDR_LE_NONE);
        profile_names[i].name[0] = '\0';
    }
#endif

    LOG_DBG("Profile names initialized");
    return 0;
}

SYS_INIT(profile_names_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
