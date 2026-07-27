# FormMap ADR-018 v2 Firmware

The ESP32 CSI node emits station-aware ADR-018 v2 by default while retaining a
compile-time legacy v1 path.

## Wire layout

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | magic `0xC5110002` little-endian |
| 4 | 1 | protocol version `2` |
| 5 | 1 | header size `48` |
| 6 | 1 | node ID |
| 7 | 1 | antenna count |
| 8 | 2 | stable station ID |
| 10 | 2 | compact link ID |
| 12 | 2 | subcarrier count |
| 14 | 1 | Wi-Fi channel |
| 15 | 1 | flags |
| 16 | 1 | RSSI signed byte |
| 17 | 1 | noise floor signed byte |
| 18 | 2 | firmware version code |
| 20 | 4 | node-global sequence |
| 24 | 4 | per-station CSI sequence |
| 28 | 8 | local monotonic timestamp microseconds |
| 36 | 8 | salted source-MAC pseudonym |
| 44 | 2 | I/Q payload length |
| 46 | 2 | CRC16-CCITT of I/Q payload |
| 48 | N | raw I/Q bytes |

## Station identity

`wifi_csi_info_t.mac` is hashed on the ESP32. Raw MAC bytes are never sent to the
Core. The 64-bit salted hash is converted to a deterministic non-zero 16-bit
station ID. A bounded LRU table maintains independent per-station sequences.

The Link Agent uses the same hash and station-ID contract. It attempts to detect
the active Wi-Fi interface MAC on Windows and Linux. Operators may override it:

```bash
ruview-pointcloud agent --name phone-a \
  --source-mac 00:11:22:33:44:55 \
  --mac-hash-salt deployment-secret
```

or pass the already computed wire hash:

```bash
ruview-pointcloud agent --name phone-a \
  --source-mac-hash 0102030405060708
```

The salt must match `CONFIG_FORMMAP_MAC_HASH_SALT` in the firmware. The built-in
default is for development only and must be replaced for deployment privacy.

## Compile-time controls

The collector provides defaults when project Kconfig does not define these
symbols:

- `CONFIG_FORMMAP_ADR018_V2` — default `1`
- `CONFIG_FORMMAP_FIRMWARE_VERSION_CODE` — default `0x0201`
- `CONFIG_FORMMAP_STATION_TABLE_SIZE` — default `16`
- `CONFIG_FORMMAP_MAC_HASH_SALT` — development-only default string

They may be overridden through compiler definitions or project-specific Kconfig.

## Flags

- bit 0: 40 MHz bandwidth
- bit 2: STBC
- bit 4: cross-node synchronization valid
- bit 5: station identity valid

## Compatibility

Core accepts ADR-018 v1, v6 and v2. Only v2 is deterministic for multiple
stations. v1/v6 Link Agent time correlation remains a lower-confidence
compatibility mode.
