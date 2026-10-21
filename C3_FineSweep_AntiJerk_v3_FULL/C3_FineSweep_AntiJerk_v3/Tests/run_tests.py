#!/usr/bin/env python3
"""Portable regression checks; optional --ecj /path/ecj.jar if javac unavailable."""
from pathlib import Path
import argparse,subprocess,tempfile
p=argparse.ArgumentParser();p.add_argument('--ecj');args=p.parse_args()
root=Path(__file__).resolve().parents[1]
def run(cmd):subprocess.run([str(c) for c in cmd],check=True,cwd=root)
with tempfile.TemporaryDirectory(prefix='response-tests-') as tmp:
 out=Path(tmp)
 for version in [2,3]:
  exe=out/('firmware'+str(version));run(['g++','-std=c++11','-O2','-DLAB_NATIVE','-DESP_ARDUINO_VERSION_MAJOR='+str(version),'-Itests/stubs','tests/firmware_test.cpp','-o',exe]);run([exe]+([out/'stream.bin'] if version==3 else []))
 sources=['Android/app/src/main/java/vn/phenikaa/responselab/TraceData.java','tests/TraceDataTest.java']
 run((['java','-jar',args.ecj,'-8','-encoding','UTF-8','-warn:none'] if args.ecj else ['javac','-encoding','UTF-8'])+['-d',out]+sources)
 run(['java','-cp',out,'vn.phenikaa.responselab.TraceDataTest',out/'stream.bin',out/'SIMULATION.csv'])
print('All software checks passed; no physical balance claim.')
