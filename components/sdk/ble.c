#include "custom_config.h"
#include "ble.h"
#include "ble_event.h"
#include "patch_tab.h"

extern void ble_common_env_init(void);
extern void ble_con_env_init(void);
extern void ble_scan_env_init(void);
extern void ble_adv_env_init(void);
extern void ble_test_evn_init(void);
extern void ble_iso_env_init(void);
extern void ble_eatt_evn_init(void);
extern void ble_mul_link_env_init(void);
extern void ble_car_key_env_init(void);
extern void ble_bt_bredr_env_init(void);
extern void ble_ranging_env_init(void);

extern void reg_ke_msg_patch_tab(ke_msg_tab_item_t *ke_msg_tab, uint16_t ke_msg_cnt);
extern void reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab_item_t *gapm_hci_evt_tab, uint16_t gapm_hci_evt_cnt);
extern void reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab_item_t *llm_hci_cmd_tab, uint16_t llm_hci_cmd_cnt);

extern uint16_t ble_stack_enable(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table);

void ble_sdk_patch_env_init(void)
{
    // register the ke msg handler for patch
    uint16_t ke_msg_cnt = sizeof(ke_msg_tab) / sizeof(ke_msg_tab_item_t);
    reg_ke_msg_patch_tab(ke_msg_tab, ke_msg_cnt);

    // register the llm hci cmd handler for patch
    uint16_t llm_hci_cmd_cnt = sizeof(llm_hci_cmd_tab) / sizeof(llm_hci_cmd_tab_item_t);
    reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab, llm_hci_cmd_cnt);

    // register the gapm hci evt handler for patch
    uint16_t gapm_hci_evt_cnt = sizeof(gapm_hci_evt_tab) / sizeof(gapm_hci_evt_tab_item_t);
    reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab, gapm_hci_evt_cnt);

    ble_common_env_init();

    #if CFG_ISO_SUPPORT
    uint16_t lli_hci_cmd_cnt = sizeof(lli_hci_cmd_tab) / sizeof(lli_hci_cmd_tab_item_t);
    reg_lli_hci_cmd_patch_tab(lli_hci_cmd_tab, lli_hci_cmd_cnt);
    #endif

    #if CFG_MAX_CONNECTIONS
    ble_con_env_init();
    //ble_adv_param_init(CFG_MAX_CONNECTIONS);
    #endif

    #if CFG_MAX_SCAN
    ble_scan_env_init();
    #endif

    #if CFG_MAX_ADVS
    ble_adv_env_init();
    #endif

    #if CFG_ISO_SUPPORT
    ble_iso_env_init();
    #endif

    #if CFG_EATT_SUPPORT
    ble_eatt_evn_init();
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    ble_mul_link_env_init();
    #endif

    #if CFG_CAR_KEY_SUPPORT
    ble_car_key_env_init();
    #endif

    #if CFG_BT_BREDR
    ble_bt_bredr_env_init();
    #endif

    #if CFG_RANGING_SUPPORT
    ble_ranging_env_init();
    #endif
}

uint16_t ble_stack_init(ble_evt_handler_t evt_handler, stack_heaps_table_t *p_heaps_table)
{
    ble_sdk_patch_env_init();
    return ble_stack_enable(evt_handler, p_heaps_table);
}
