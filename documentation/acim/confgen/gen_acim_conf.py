#!/usr/bin/env python3
"""Generate motor/acim_confgen.c and .h from documentation/acim/acim_settings.xml.

    python3 gen_acim_conf.py          # write the two files
    python3 gen_acim_conf.py --check  # exit 1 if the committed files differ from the output

The XML is the ACIM page as VESC Tool sees it. From it this writes the config struct, the
defaults, the serializer (in <SerOrder>), the page signature and the qCompress()ed XML that the
firmware sends to VESC Tool. Edit the XML, never the generated files.

Signature: VESC Tool's ConfigParams::getSignature(), CRC32C over, for each serialized param,
name + type + vTx + enum names. Params without a <vTx> (bool, enum) count as vTx 0.
"""
import os
import sys
import zlib
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
XML = os.path.join(ROOT, "documentation", "acim", "acim_settings.xml")
OUT_C = os.path.join(ROOT, "motor", "acim_confgen.c")
OUT_H = os.path.join(ROOT, "motor", "acim_confgen.h")
GENERATED_BY = "documentation/acim/confgen/gen_acim_conf.py"

# ConfigParam types and transmit types used by VESC Tool
T_DOUBLE, T_INT, T_ENUM, T_BOOL = 1, 2, 4, 5
TX_UINT8, TX_INT8, TX_UINT16, TX_INT16, TX_UINT32, TX_INT32, TX_DOUBLE16, TX_DOUBLE32, TX_DOUBLE32_AUTO = range(1, 10)

LICENSE = """/*
	Copyright 2026 wiredsim

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
"""


def crc32c(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 & -(crc & 1))
    return ~crc & 0xFFFFFFFF


def load():
    xml_bytes = open(XML, "rb").read()
    root = ET.fromstring(xml_bytes)
    params = {p.tag: p for p in root.find("Params")}
    order = [e.text for e in root.find("SerOrder")]
    out = []
    for name in order:
        p = params[name]
        vtx = p.find("vTx")
        out.append(dict(
            name=name,
            type=int(p.find("type").text),
            vtx=int(vtx.text) if vtx is not None else 0,
            cdef=p.find("cDefine").text,
            enums=[e.text for e in p.findall("enumNames")],
            val_int=p.findtext("valInt"),
            val_double=p.findtext("valDouble"),
            scale=p.findtext("vTxDoubleScale"),
        ))
    return xml_bytes, out


def signature(params):
    s = "".join(p["name"] + str(p["type"]) + str(p["vtx"]) + "".join(p["enums"]) for p in params)
    return crc32c(s.encode("utf-8"))


def qcompress(data):
    # Qt's qCompress: 4-byte big-endian uncompressed length, then a zlib stream
    return len(data).to_bytes(4, "big") + zlib.compress(data, 9)


def c_float(text):
    return repr(float(text)) + "f"


def ctype(p):
    if p["type"] == T_BOOL:
        return "bool"
    if p["type"] == T_ENUM:
        return "uint8_t"
    if p["type"] == T_DOUBLE:
        return "float"
    if p["type"] == T_INT:
        return {TX_UINT8: "uint8_t", TX_INT8: "int8_t", TX_UINT16: "uint16_t", TX_INT16: "int16_t",
                TX_UINT32: "uint32_t", TX_INT32: "int32_t"}[p["vtx"]]
    raise SystemExit("unsupported type %d for %s" % (p["type"], p["name"]))


def default(p):
    if p["type"] == T_BOOL:
        return "true" if int(p["val_int"]) else "false"
    if p["type"] in (T_ENUM, T_INT):
        return str(int(p["val_int"]))
    return c_float(p["val_double"])


def ser(p):
    n = "conf->" + p["name"]
    if p["type"] in (T_BOOL, T_ENUM):
        return "\tbuffer[ind++] = (uint8_t)%s;" % n
    if p["type"] == T_DOUBLE:
        if p["vtx"] == TX_DOUBLE32_AUTO:
            return "\tbuffer_append_float32_auto(buffer, %s, &ind);" % n
        if p["vtx"] in (TX_DOUBLE16, TX_DOUBLE32):
            bits = 16 if p["vtx"] == TX_DOUBLE16 else 32
            return "\tbuffer_append_float%d(buffer, %s, %s, &ind);" % (bits, n, p["scale"])
    if p["type"] == T_INT:
        fn = {TX_UINT8: None, TX_INT8: None, TX_UINT16: "uint16", TX_INT16: "int16", TX_UINT32: "uint32", TX_INT32: "int32"}[p["vtx"]]
        return "\tbuffer[ind++] = (uint8_t)%s;" % n if fn is None else "\tbuffer_append_%s(buffer, %s, &ind);" % (fn, n)
    raise SystemExit("unsupported transmit type for %s" % p["name"])


def deser(p):
    n = "conf->" + p["name"]
    if p["type"] == T_BOOL:
        return "\t%s = buffer[ind++] != 0;" % n
    if p["type"] == T_ENUM:
        return "\t%s = buffer[ind++];" % n
    if p["type"] == T_DOUBLE:
        if p["vtx"] == TX_DOUBLE32_AUTO:
            return "\t%s = buffer_get_float32_auto(buffer, &ind);" % n
        bits = 16 if p["vtx"] == TX_DOUBLE16 else 32
        return "\t%s = buffer_get_float%d(buffer, %s, &ind);" % (n, bits, p["scale"])
    if p["type"] == T_INT:
        fn = {TX_UINT8: None, TX_INT8: None, TX_UINT16: "uint16", TX_INT16: "int16", TX_UINT32: "uint32", TX_INT32: "int32"}[p["vtx"]]
        return "\t%s = buffer[ind++];" % n if fn is None else "\t%s = buffer_get_%s(buffer, &ind);" % (n, fn)
    raise SystemExit("unsupported transmit type for %s" % p["name"])


def generate():
    xml_bytes, params = load()
    blob = qcompress(xml_bytes)
    sig = signature(params)

    h = [LICENSE, "// This file is generated by %s. Do not edit.\n" % GENERATED_BY,
         "#ifndef ACIM_CONFGEN_H_", "#define ACIM_CONFGEN_H_", "",
         "#include <stdint.h>", "#include <stdbool.h>", "",
         "#define ACIM_CONF_SIGNATURE\t\t%du" % sig,
         "#define ACIM_CONF_XML_SIZE\t\t%d" % len(blob), ""]
    for p in params:
        if p["type"] == T_ENUM:
            h.append("typedef enum {")
            h.append(",\n".join("\t%s_%s" % (p["cdef"], e.upper().replace(" ", "_")) for e in p["enums"]))
            h += ["} acim_%s;" % p["name"], ""]
    h.append("typedef struct {")
    h += ["\t%s %s;" % (ctype(p), p["name"]) for p in params]
    h += ["} acim_config;", "",
          "extern const uint8_t acim_conf_xml[ACIM_CONF_XML_SIZE];", "",
          "void acim_confgen_set_defaults(acim_config *conf);",
          "int32_t acim_confgen_serialize(uint8_t *buffer, const acim_config *conf);",
          "bool acim_confgen_deserialize(const uint8_t *buffer, acim_config *conf);", "",
          "#endif /* ACIM_CONFGEN_H_ */", ""]

    c = [LICENSE, "// This file is generated by %s. Do not edit.\n" % GENERATED_BY,
         '#include "acim_confgen.h"', '#include "buffer.h"', "",
         "void acim_confgen_set_defaults(acim_config *conf) {"]
    c += ["\tconf->%s = %s;" % (p["name"], default(p)) for p in params]
    c += ["}", "", "int32_t acim_confgen_serialize(uint8_t *buffer, const acim_config *conf) {",
          "\tint32_t ind = 0;", "\tbuffer_append_uint32(buffer, ACIM_CONF_SIGNATURE, &ind);"]
    c += [ser(p) for p in params]
    c += ["\treturn ind;", "}", "", "bool acim_confgen_deserialize(const uint8_t *buffer, acim_config *conf) {",
          "\tint32_t ind = 0;", "", "\tif (buffer_get_uint32(buffer, &ind) != ACIM_CONF_SIGNATURE) {",
          "\t\treturn false;", "\t}", ""]
    c += [deser(p) for p in params]
    c += ["", "\treturn true;", "}", "",
          "// qCompress() of documentation/acim/acim_settings.xml",
          "const uint8_t acim_conf_xml[ACIM_CONF_XML_SIZE] = {"]
    for i in range(0, len(blob), 16):
        c.append("\t" + " ".join("0x%02x," % b for b in blob[i:i + 16]))
    c += ["};", ""]
    return "\n".join(h), "\n".join(c), sig


def main():
    h, c, sig = generate()
    if "--check" in sys.argv:
        ok = open(OUT_H).read() == h and open(OUT_C).read() == c
        print("generated files are up to date (signature %d)" % sig if ok else "generated files differ from the XML")
        sys.exit(0 if ok else 1)
    open(OUT_H, "w").write(h)
    open(OUT_C, "w").write(c)
    print("wrote %s and %s, signature %d" % (OUT_H, OUT_C, sig))


if __name__ == "__main__":
    main()
