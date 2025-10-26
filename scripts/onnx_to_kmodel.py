#!/usr/bin/env python3
"""
ONNX -> KModel converter for K230 using nncase.

Usage examples:
  python scripts/onnx_to_kmodel.py \
    --onnx best.onnx \
    --out K230/sdcard/best.kmodel \
    --shape 1,3,320,320 \
    --input-type uint8 \
    --target k230 \
    --plugin-dir F:\\K230\\APP\\nncase-master

Notes:
- Requires: pip install nncase (please connect to internet when installing)
- If your model expects 640x640 input, set --shape 1,3,640,640
- For calibration-based quantization you need extra options/dataset (not covered here)
"""
import argparse
import os
import sys


def parse_shape(s: str):
    parts = [int(p.strip()) for p in s.split(',') if p.strip()]
    if len(parts) != 4:
        raise ValueError("--shape must be 'N,C,H,W', e.g., 1,3,320,320")
    return parts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--onnx', required=True, help='Path to ONNX model file')
    parser.add_argument('--out', required=True, help='Path to output .kmodel file')
    parser.add_argument('--shape', default='1,3,320,320', help='Input shape N,C,H,W (default: 1,3,320,320)')
    parser.add_argument('--input-type', default='uint8', choices=['uint8', 'float32'], help='Input type (default uint8)')
    parser.add_argument('--target', default='k230', help="Compile target (default 'k230')")
    parser.add_argument('--plugin-dir', default=None, help='NNCASE plugin directory (overrides NNCASE_PLUGIN_PATH)')
    args = parser.parse_args()

    # Ensure plugin path is set before importing nncase
    if args.plugin_dir:
        os.environ['NNCASE_PLUGIN_PATH'] = args.plugin_dir
    elif 'NNCASE_PLUGIN_PATH' not in os.environ:
        # Try common default on Windows per our project convention
        default_plugin = r'F:\\K230\\APP\\nncase-master'
        if os.path.isdir(default_plugin):
            os.environ['NNCASE_PLUGIN_PATH'] = default_plugin

    # Now import nncase after env is prepared
    try:
        import nncase
        from nncase import CompileOptions, Compiler, ImportOptions
    except Exception as e:
        print("[ERROR] nncase is not installed or failed to load. Run: pip install nncase", file=sys.stderr)
        raise

    input_shape = parse_shape(args.shape)

    # Prepare compile options
    cpl_opt = CompileOptions()
    cpl_opt.target = args.target
    cpl_opt.input_shape = input_shape
    cpl_opt.input_type = args.input_type

    imp_opt = ImportOptions()
    imp_opt.input_shapes = [input_shape]

    # Create compiler and import onnx
    compiler = Compiler(cpl_opt)
    with open(args.onnx, 'rb') as f:
        compiler.import_onnx(f.read(), imp_opt)

    # Compile and dump kmodel
    compiler.compile()
    kmodel_bytes = compiler.gencode_tobytes()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'wb') as f:
        f.write(kmodel_bytes)

    print(f"[OK] KModel saved to: {args.out}")


if __name__ == '__main__':
    main()

