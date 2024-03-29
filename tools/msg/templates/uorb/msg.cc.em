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
@#  - constrained_flash set to true if flash is constrained
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
topic_name = spec.short_name

sorted_fields = sorted(spec.parsed_fields(), key=sizeof_field_type, reverse=True)
struct_size, padding_end_size = add_padding_bytes(sorted_fields, search_path)
topic_fields = ["%s %s" % (convert_type(field.type), field.name) for field in sorted_fields]

topic_queue_size = 1
for constant in spec.constants:
  if constant.name == "ORB_QUEUE_SIZE":
    topic_queue_size =  constant.val
    break
}@

#include <uorb/topics/@(topic_name).h>

@# join all msg files in one line e.g: "float[3] position;float[3] velocity;bool armed"
@# This is used for the logger
static constexpr char orb_@(topic_name)_fields[] =
  "@( ";".join(topic_fields) );";

@[for multi_topic in topics]@
ORB_DEFINE(@multi_topic, struct @uorb_struct, @(struct_size-padding_end_size), orb_@(topic_name)_fields, @topic_queue_size);
@[end for]
