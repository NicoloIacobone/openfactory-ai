#!/usr/bin/env python3
"""Copy a take from the VM and edit it with ffmpeg: the Mac side of the filming tool (D-029).

A take is recorded in the VM by scripts/film.py on simulated time; here it is copied into the
git-ignored data/videos/ and sped up, cut or turned into a GIF. The recording is never changed:
every edit is a new file with its speed in the name, so an edit can be redone at will.

Run on the Mac, from anywhere (needs ffmpeg and SSH access to the machine that filmed the take):
    python3 scripts/film_edit.py fetch NAME [--host HOST]
    python3 scripts/film_edit.py info NAME
    python3 scripts/film_edit.py edit NAME --speed 8 [--trim START END] [--fps 30] [--gif [WIDTH]]
    python3 scripts/film_edit.py frames NAME [--count 5]
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import film_shots as fs

REPO = Path(__file__).resolve().parents[1]
VIDEOS = REPO / 'data' / 'videos'
DEFAULT_HOST = 'doppelvm'           # SSH host of the development VM (docs/environment.md)
VM_VIDEOS = 'openfactory_data/videos'   # where film.py writes takes, relative to the home directory
GIF_WIDTH = 640
GIF_FPS = 12


def recording(name):
    path = VIDEOS / name / f'{name}.{fs.VIDEO_FORMAT}'
    if not path.exists():
        sys.exit(f'{path} is not there: python3 scripts/film_edit.py fetch {name}')
    return path


def probe(path):
    """{'width', 'height', 'codec', 'fps', 'duration_s', 'frames'} of a video file."""
    out = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-count_packets', '-show_entries',
         'stream=width,height,codec_name,avg_frame_rate,nb_read_packets:format=duration',
         '-of', 'json', str(path)], capture_output=True, text=True, check=True).stdout
    data = json.loads(out)
    stream = data['streams'][0]
    numerator, _, denominator = stream['avg_frame_rate'].partition('/')
    return {'width': stream['width'], 'height': stream['height'], 'codec': stream['codec_name'],
            'fps': round(float(numerator) / float(denominator or 1), 3) if float(numerator) else 0,
            'duration_s': round(float(data['format']['duration']), 3),
            'frames': int(stream['nb_read_packets'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    fetch = sub.add_parser('fetch', help='copy a take from the machine that filmed it into data/videos/')
    fetch.add_argument('name')
    fetch.add_argument('--host', default=DEFAULT_HOST,
                       help=f'SSH host (alias or user@address) that filmed the take '
                            f'(default {DEFAULT_HOST})')
    sub.add_parser('info', help='the recording next to what take.json says').add_argument('name')
    edit = sub.add_parser('edit', help='speed up, cut, convert')
    edit.add_argument('name')
    edit.add_argument('--speed', type=float, default=1.0)
    edit.add_argument('--trim', type=float, nargs=2, metavar=('START', 'END'),
                      help='seconds of the recording to keep')
    edit.add_argument('--fps', type=int, help=f'of the edit (default 30; {GIF_FPS} for a GIF)')
    edit.add_argument('--gif', type=int, nargs='?', const=GIF_WIDTH, metavar='WIDTH',
                      help=f'write a GIF this wide (default {GIF_WIDTH}) instead of an mp4')
    frames = sub.add_parser('frames', help='evenly spaced frames of the recording, as PNG')
    frames.add_argument('name')
    frames.add_argument('--count', type=int, default=5)
    args = parser.parse_args()

    if args.command == 'fetch':
        VIDEOS.mkdir(parents=True, exist_ok=True)
        subprocess.run(['scp', '-r', '-q', f'{args.host}:{VM_VIDEOS}/{args.name}', str(VIDEOS)],
                       check=True)
        print(VIDEOS / args.name)
        return 0

    source = recording(args.name)
    if args.command == 'info':
        take_file = source.parent / 'take.json'
        take = json.loads(take_file.read_text()) if take_file.exists() else {}
        print(json.dumps({'file': probe(source), 'take': {
            key: take.get(key) for key in ('take', 'git_commit', 'params', 'sim_duration_s',
                                           'success')}}, indent=2))
    elif args.command == 'edit':
        extension = 'gif' if args.gif else 'mp4'
        destination = source.parent / fs.edit_name(args.name, args.speed, args.trim, extension)
        fps = args.fps or (GIF_FPS if args.gif else 30)
        subprocess.run(fs.ffmpeg_args(source, destination, args.speed, args.trim, fps, args.gif),
                       check=True)
        print(f"{destination}: {json.dumps(probe(destination))}")
    else:
        duration = probe(source)['duration_s']
        out_dir = source.parent / 'frames'
        out_dir.mkdir(exist_ok=True)
        for index in range(args.count):
            at = duration * (index + 0.5) / args.count
            path = out_dir / f'{args.name}_{at:07.1f}s.png'
            subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-ss',
                            f'{at:.2f}', '-i', str(source), '-frames:v', '1', str(path)],
                           check=True)
            print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
