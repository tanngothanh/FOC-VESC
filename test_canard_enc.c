#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "canard.h"
#include "uavcan/equipment/esc/RawCommand.h"

int main() {
    uint8_t buf[16] = {0};
    uavcan_equipment_esc_RawCommand cmd;
    int16_t data[2] = {0, 1400};
    cmd.cmd.len = 2;
    cmd.cmd.data = data;

    uint32_t len = uavcan_equipment_esc_RawCommand_encode(&cmd, buf);
    printf("Encoded len: %u bytes\n", (unsigned int)len);
    for (uint32_t i = 0; i < len; i++) {
        printf("%02Z ", buf[i]);
    }
    printf("\n");

    CanardRxTransfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.payload_head = buf;
    tr.payload_len = (uint16_t)len;

    uavcan_equipment_esc_RawCommand dcmd;
    uint8_t dyn[64];
    uint8_t *tmp = dyn;
    int32_t res = uavcan_equipment_esc_RawCommand_decode_internal(&tr, (uint16_t)len, &dcmd, &tmp, 0);
    printf("Decoded res: %d, len: %u, d[0]: %d, d[1]: %d\n", (int)res, (unsigned int)dcmd.cmd.len, (int)dcmd.cmd.data[0], (int)dcmd.cmd.data[1]);
    return 0;
}
