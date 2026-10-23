"""Write a no-deadzone profile for the gesture wheel only (verified v3 ABI)."""
from pathlib import Path
import struct
import sys
import tomllib


def profile(config):
    names = dict(south=0,east=1,west=2,north=3,back=4,guide=5,start=6,
                 left_stick=7,right_stick=8,left_shoulder=9,right_shoulder=10,
                 dpad_up=11,dpad_down=12,dpad_left=13,dpad_right=14)
    defaults = ['east','south','north','west','start','back','left_shoulder',
                'right_shoulder','dpad_up','dpad_down','dpad_left','dpad_right']
    keys = ['a','b','x','y','start','z','l','r','up','down','left','right']
    gc = [0x100,0x200,0x400,0x800,0x1000,0x10,0x40,0x20,8,4,1,2]
    data = bytearray(struct.pack('<II?',0x4c525443,3,False)+bytes(23))
    # emulateTriggers, useDeadzones, stick/substick deadzone, L/R threshold.
    data += struct.pack('<??4H',False,False,0,0,31150,31150)
    for key,default,button in zip(keys,defaults,gc):
        name=config.get(key,default)
        if name not in names:
            raise ValueError('Unsupported existing binding: '+name)
        data += struct.pack('<IH2x',names[name],button)
    for i,(axis,sign) in enumerate([(0,1),(0,-1),(1,-1),(1,1),
                                   (2,1),(2,-1),(3,-1),(3,1),(4,1),(5,1)]):
        data += struct.pack('<iiiH2x',axis,sign,-1,i)
    data += struct.pack('<HH',0,0)
    assert len(data)==302
    return data


if __name__=='__main__':
    root=Path(sys.argv[1])
    # SDL's gamepad mapping exposes the Xbox name, not the kernel device name.
    path=root/'Xbox 360 Controller_045E_028E.controller'
    if path.exists():
        raise SystemExit('Profile already exists; preserve/review it before replacing')
    config=tomllib.loads((root/'Config.toml').read_text()).get('controller',{})
    temp=path.with_suffix('.pending')
    temp.write_bytes(profile(config))
    temp.replace(path)
    print('Created gesture-wheel-only no-deadzone profile')
