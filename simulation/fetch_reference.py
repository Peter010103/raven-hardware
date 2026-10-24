"""Fetch hash-verified firmware reference sources from the pinned upstream commit."""
import hashlib,json,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    manifest=json.loads((ROOT/'simulation/firmware_reference.json').read_text());out=ROOT/'build/betaflight-4.5.5';out.mkdir(parents=True,exist_ok=True)
    for record in manifest['files']:
        data=urllib.request.urlopen(record['url'],timeout=30).read()
        if hashlib.sha256(data).hexdigest()!=record['SHA256']:raise ValueError('Upstream source hash mismatch')
        (out/record['path'].replace('src/main/','').replace('/','_')).write_bytes(data)
    print(f'Fetched {len(manifest["files"])} verified source files at {manifest["commit"]}')

if __name__=='__main__':main()
