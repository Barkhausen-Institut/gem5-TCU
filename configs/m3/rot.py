import os
import sys

sys.path.append(os.path.realpath('platform/gem5/configs/m3'))  # NOQA
from base import *

options = getOptions()
root = createRoot(options)

# cmd is only used to extract the kernel cmdline
cmd_list = options.cmd.split(",")
kernel_cmdline = cmd_list[0]
assert all(cmd == '' for cmd in cmd_list[1:]), "cmdline must be empty for all except the first kernel tile"

rot_layers = options.rot_layers.split(",")
assert rot_layers[0], "must specify at least one RoT layer"

num_mem = 1
num_flash = 1
num_tiles = int(os.environ.get('M3_GEM5_TILES'))
accs = ['copy', 'copy', 'copy']
# FFT, MUL, IFFT times for a 4K block
comptimes = [5856 * 2, 1189 * 2, 5856 * 2]
rot_cpu = os.environ.get('M3_GEM5_ROT_CPU')

kecaccspmsize = '128MiB'
if os.environ.get('M3_ISA') == 'riscv32':
    l1size=None
    l2size=None
    spmsize='128MiB'
else:
    l1size='32KiB'
    l2size='128KiB'
    spmsize=None

hard_disk0 = os.environ.get('M3_GEM5_IDE_DRIVE')

mem_tile = None
mem_tile_id = None
flash_tile = None
flash_tile_id = None

tiles = []

# create the memory tiles
for i in range(0, num_mem):
    tile_id = TileId(0, len(tiles))
    tile = createMemTile(noc=root.noc,
                         options=options,
                         id=tile_id,
                         size='3GiB')
    tiles.append(tile)
    mem_tile = mem_tile or tile
    mem_tile_id = mem_tile_id or tile_id

# create the flash tiles
for i in range(0, num_flash):
    tile_id = TileId(0, len(tiles))
    tile = createMemTile(noc=root.noc,
                         options=options,
                         id=tile_id,
                         size=os.environ.get('M3_GEM5_FLASH_SIZE', '512MiB'),
                         memType='Flash')
    tiles.append(tile)
    flash_tile = flash_tile or tile
    flash_tile_id = flash_tile_id or tile_id

for i in range(0, num_tiles):
    tile = createCoreTile(noc=root.noc,
                          options=options,
                          id=TileId(0, len(tiles)),
                          cmdline='',
                          memTile=None,
                          l1size=l1size,
                          l2size=l2size,
                          spmsize=spmsize)
    tiles.append(tile)

old_cpu = options.cpu_type
if rot_cpu is not None:
    options.cpu_type = rot_cpu
tile = createRoTTile(noc=root.noc,
                     options=options,
                     id=TileId(0, len(tiles)),
                     cmdline=rot_layers[0],
                     rotLayers=rot_layers[1:],
                     kernelCmdline=kernel_cmdline,
                     flashTile=flash_tile,
                     # flashTile=mem_tile,  # RoT can also run fetch data directly from memory tile
                     # FIXME: The RoT boot layers run with 256 KiB but TileMux/RoTS currently need more
                     spmsize=os.environ.get('M3_GEM5_ROT_MEMSIZE', kecaccspmsize))
options.cpu_type = old_cpu
tiles.append(tile)

# create accelerator tiles
for i in range(0, len(accs)):
    tile = createAccelTile(noc=root.noc,
                           options=options,
                           id=TileId(0, len(tiles)),
                           accel=accs[i],
                           memTile=None,
                           spmsize='32MiB',
                           comptime=comptimes[i])
    tiles.append(tile)

# create the persistent storage tile
if hard_disk0 and os.path.isfile(hard_disk0):
    tile = createStorageTile(noc=root.noc,
                             options=options,
                             id=TileId(0, len(tiles)),
                             memTile=None,
                             img0=hard_disk0)
    tiles.append(tile)#
else:
    print('NOTE: Skipping persistent storage tile because no hard disk image was specified')
    print()

# create ether tile connected to the host
if os.path.isdir("/sys/class/net/gem5-tap"):
    ether = createEtherTile(noc=root.noc,
                             options=options,
                             id=TileId(0, len(tiles)),
                             memTile=None)
    tiles.append(ether)
    ether.ethertap = EtherTap(tap=ether.nic.interface)
else:
    print('NOTE: Skipping ether tile because no gem5-tap interface was found')
    print('Use the following commands to set it up:')
    print('\t$ sudo ip tuntap add gem5-tap mode tap')
    print('\t$ sudo ip link set gem5-tap up')
    print('\t$ sudo ip addr add 192.168.42.2/24 dev gem5-tap')
    print()

# create tile for serial input
tile = createSerialTile(noc=root.noc,
                        options=options,
                        id=TileId(0, len(tiles)),
                        memTile=None)
tiles.append(tile)

runSimulation(root, options, tiles)
