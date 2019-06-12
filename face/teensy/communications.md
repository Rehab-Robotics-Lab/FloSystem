# Communication Protocol

## Things we need to control for now:
2x eyes - 64 points = 8 bytes
1x mouth - 128 points = 16 bytes
1x servo - ??

## Packet design

| Control Sequence | Data     | End Cap |
|-----------------|----------|---------|
| 1 byte          | 16 bytes | 1 byte  |

### Control Codes

| Code | Command |
|------|---------|
| 0x1  | Initialize |
| 0x2  | Write Mouth |
| 0x3  | Write Left Eye |
| 0x4  | Write Right Eye |
| 0x5  | Write neck/servo |