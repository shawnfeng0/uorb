@###############################################
@#
@# PX4 ROS compatible message source code
@# generation for C++
@#
@# EmPy template for generating <msg>.h files
@# Based on the original template for ROS
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - file_name_in (String) Source file
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - md5sum (String) MD5Sum of the .msg specification
@#  - search_path (dict) search paths for genmsg
@#  - topics (List of String) multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

@{
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/

uorb_struct = '%s_s'%spec.short_name
uorb_struct_upper = spec.short_name.upper()
topic_name = spec.short_name
}@

#pragma once

@##############################
@# Generic Includes
@##############################

#include <uorb/uORB.h>
#include <inttypes.h>

@##############################
@# Includes for dependencies
@##############################
@{
for field in spec.parsed_fields():
    if (not field.is_builtin):
        if (not field.is_header):
            (package, name) = genmsg.names.package_resource_name(field.base_type)
            package = package or spec.package # convert '' to package
            print('#include <uorb/topics/%s.h>'%(name))
}@

/* register this as object request broker structure */
@[for multi_topic in topics]@
ORB_DECLARE(@multi_topic);
@[end for]

@# Constants c style
#ifndef __cplusplus
@[for constant in spec.constants]@
#define @(uorb_struct_upper)_@(constant.name) @(int(constant.val))
@[end for]
#endif

@##############################
@# Main struct of message
@##############################
@{

def print_parsed_fields():
    # sort fields (using a stable sort)
    sorted_fields = sorted(spec.parsed_fields(), key=sizeof_field_type, reverse=True)
    struct_size, padding_end_size = add_padding_bytes(sorted_fields, search_path)
    # loop over all fields and print the type and name
    for field in sorted_fields:
        if (not field.is_header):
            print_field_def(field)
}@

#ifdef __cplusplus
@#class @(uorb_struct) {
struct __EXPORT @(uorb_struct) {
@#public:
#else
struct @(uorb_struct) {
#endif
@print_parsed_fields()

#ifdef __cplusplus
@# Constants again c++-ified
@{
for constant in spec.constants:
    type_name = constant.type
    if type_name in type_map:
        # need to add _t: int8 --> int8_t
        type_px4 = type_map[type_name]
    else:
        raise Exception("Type {0} not supported, add to to template file!".format(type_name))

    print('\tstatic constexpr %s %s = %s;'%(type_px4, constant.name, int(constant.val)))
}
@{
print('\tstatic inline const constexpr orb_metadata &get_metadata() {')
print('\t\treturn *ORB_ID(%s);'%(topic_name))
print('\t}')
}
#endif
};