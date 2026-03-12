# Hall Indexing Mode — C/C++ Changes & Python API Update Guide

## Overview

The `Hall_Indexing_mode` was extended to support edge detection (`trigger_value = 2`) in addition to the existing level-based triggering (`trigger_value = 0` or `1`). The CAN protocol for the `IN_HALL_INDEX` command and `OUT_DATA_HALL` response was updated accordingly.

---

## 1. New Controller Struct Fields (`common.h`)

Three new fields were added to the controller struct:

| Field | Type | Default | Purpose |
|---|---|---|---|
| `hall_index` | `volatile bool` | `0` | Set to `1` when edge is detected, cleared on re-arm |
| `additional2_var_prev` | `volatile bool` | `0` | Internal — previous Hall pin state for edge detection |
| `hall_edge_needs_init` | `volatile bool` | `1` | Internal — prevents false trigger on re-entry into mode 8 |

`trigger_value` type was widened from `volatile bool` to `volatile uint8_t` to support value `2`.

---

## 2. `IN_HALL_INDEX` — CAN Receive (Python → Driver)

### Old protocol
`data[3]` was a bitfield byte — `trigger_value` was extracted as bit 7 (`bitArray[0]`), so Python packed it as a bitfield. Only values `0` and `1` were supported.

### New protocol
`data[3]` is a **raw byte** used directly as `trigger_value`:

| Value | Meaning |
|---|---|
| `0` | Trigger when pin is LOW |
| `1` | Trigger when pin is HIGH |
| `2` | Trigger on any edge (rising or falling) |

### Re-arm behaviour
Re-arm (`hall_trigger=1`, `hall_edge_needs_init=1`, `hall_index=0`) only happens when **transitioning into mode 8**. If the driver is already in mode 8 (e.g. host is polling repeatedly), the trigger state is preserved. This allows the host to spam this command and read back results without accidentally clearing a detected edge.

### Packet format (4 bytes, unchanged)

```
data[0..2] : velocity setpoint while searching (3-byte signed int)
data[3]    : trigger_value raw byte (0, 1, or 2)
```

---

## 3. `OUT_DATA_HALL` — CAN Response (Driver → Python)

### Old status byte (`data[3]`) bitfield

| Bit | Field |
|---|---|
| 7 | `hall_trigger` |
| 6 | `additional_pin2` |
| 5..0 | unused (0) |

### New status byte (`data[3]`) bitfield

| Bit | Field |
|---|---|
| 7 | `hall_trigger` — `1` = still waiting, `0` = triggered/latched |
| 6 | `additional_pin2` — raw current pin state |
| 5 | `hall_index` — `1` = edge was detected since last arm |
| 4..0 | unused (0) |

### Packet format (4 bytes, unchanged)

```
data[0..2] : current position (3-byte signed int)
data[3]    : status bitfield as above
```

---

## 4. Python API Changes Required

### 4.1 Send function — `Send_data_pack_HALL`

Replace bitfield packing with a raw byte:

```python
# OLD
def Send_data_pack_HALL(self, Speed=0, trigger_value=0):
    _canid = util.combine_2_can_id(self.node_id, self.SEND_HALL_INDEX, 0)
    _speed = util.split_2_3_bytes(Speed)
    bitfield_list = [trigger_value, 0, 0, 0, 0, 0, 0, 0]
    fused = util.fuse_bitfield_2_bytearray(bitfield_list)
    combined = _speed + fused
    _combined = bytearray(combined)
    self.communication.send_can_message(message_id=_canid, data=_combined)

# NEW
def Send_data_pack_HALL(self, Speed=0, trigger_value=0):
    _canid = util.combine_2_can_id(self.node_id, self.SEND_HALL_INDEX, 0)
    _speed = util.split_2_3_bytes(Speed)
    # trigger_value sent as raw byte: 0=low level, 1=high level, 2=any edge
    combined = _speed + bytearray([trigger_value])
    self.communication.send_can_message(message_id=_canid, data=combined)
```

### 4.2 Receive handler — `RESPOND_DATA_HALL`

Extract the new `hall_index` field from bit position 2 of the status byte:

```python
# OLD
case self.RESPOND_DATA_HALL:
    if message.dlc == 4:
        self.position = util.fuse_3_bytes(b'\x00' + message.data[0:3])
        byte1 = util.split_2_bitfield(message.data[3])
        self.HALL_trigger = byte1[0]
        self.additional_pin2 = byte1[1]
    else:
        self.DLC_warrning = 1

# NEW
case self.RESPOND_DATA_HALL:
    if message.dlc == 4:
        self.position = util.fuse_3_bytes(b'\x00' + message.data[0:3])
        byte1 = util.split_2_bitfield(message.data[3])
        self.HALL_trigger = byte1[0]
        self.additional_pin2 = byte1[1]
        self.hall_index = byte1[2]  # edge detected flag
    else:
        self.DLC_warrning = 1
```

### 4.3 Motor object initialisation

Add `hall_index` to wherever other state variables are initialised:

```python
self.hall_index = None  # None = no data yet, 0 = no edge, 1 = edge triggered
```

### 4.4 Downstream processing (if applicable)

Where `RESPOND_DATA_HALL` results are processed, extract `hall_index`:

```python
elif command_id == 32:  # RESPOND_DATA_HALL
    process_Motor_data(...)
    Hall_index = motor_obj.HALL_trigger
    additional_pin2 = motor_obj.additional_pin2
    hall_index = motor_obj.hall_index  # edge detected flag (None/0/1)
```

---

## 5. Usage Example

```python
# Start homing — spin at 6000 ticks/s until any edge is detected
motors[5].Send_data_pack_HALL(Speed=6000, trigger_value=2)

# Poll in loop — safe to spam, trigger state is preserved
while True:
    motors[5].Send_data_pack_HALL(Speed=6000, trigger_value=2)
    if motors[5].hall_index == 1:
        # Edge detected — driver is now holding position
        break

# Move away in velocity mode, then re-home
# Switching to any other mode and back to mode 8 will automatically re-arm
```
