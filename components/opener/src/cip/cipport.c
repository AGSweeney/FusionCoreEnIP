/*******************************************************************************
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/
/** @file cipport.c
 *  @brief Implement the CIP Port Object (Class 0xF4)
 *
 *  CIP Port Object
 *  ===============
 *
 *  The Port Object represents a physical communication interface (port) on an
 *  EtherNet/IP device. It provides port-level information and diagnostics.
 *
 *  Implemented Attributes
 *  ----------------------
 *  - Attribute 1: Port Type (USINT) - Type of port (e.g., TCP=1)
 *  - Attribute 2: Port Number (USINT) - Unique port number
 *  - Attribute 3: Link Path (EPATH) - Path to associated link object
 *  - Attribute 4: Port Name (SHORT_STRING) - Human-readable port name
 *  - Attribute 5: Port Type Name (SHORT_STRING) - Port type description
 *  - Attribute 6: Node Address (USINT) - Node address on network (optional)
 *
 *  Implemented Services
 *  --------------------
 *  - GetAttributeAll
 *  - GetAttributeSingle
 */

#include "cipport.h"

#include <string.h>

#include "opener_user_conf.h"
#include "cipcommon.h"
#include "cipmessagerouter.h"
#include "ciperror.h"
#include "cipstring.h"
#include "endianconv.h"
#include "ciptcpipinterface.h"
#include "opener_api.h"
#include "trace.h"

/** @brief Number of Port Object instances */
#define PORT_INSTANCE_COUNT 1

/** @brief Port Object instance data */
CipPortObject g_port[PORT_INSTANCE_COUNT] = {
  {
    .port_type = kPortTypeTCP,                    /* Attribute #1: TCP port */
    .port_number = 1,                             /* Attribute #2: Port number 1 */
    .link_path = {                                /* Attribute #3: Link path to TCP/IP Interface */
      2,                                          /* PathSize in 16-bit chunks */
      kCipTcpIpInterfaceClassCode,                 /* Class ID (0xF5) */
      1,                                          /* Instance Number #1 */
      0                                           /* Attribute Number (not used) */
    },
    .port_name = {0, NULL},                       /* Attribute #4: Will be set in init */
    .port_type_name = {0, NULL},                 /* Attribute #5: Will be set in init */
    .node_address = 0                             /* Attribute #6: Node address (0 = not used) */
  }
};

/* Static Functions - none needed, using standard EncodeCipEPath */

/* Public Functions */

/** @brief Initialize the Port Object
 *
 *  @return kEipStatusOk if initialization was successful, otherwise kEipStatusError
 */
EipStatus CipPortInit(void) {
  CipClass *port_class = NULL;
  
  /* Create the Port Object class */
  port_class = CreateCipClass(
    kCipPortClassCode,           /* class code */
    0,                           /* # of non-default class attributes */
    7,                           /* # highest class attribute number (default attributes 1-7) */
    2,                           /* # of class services */
    6,                           /* # of instance attributes */
    6,                           /* # highest instance attribute number */
    2,                           /* # of instance services (GetAttributeSingle, GetAttributeAll) */
    PORT_INSTANCE_COUNT,         /* # of instances */
    "Port",                      /* class name */
    1,                           /* class revision */
    NULL                         /* initialization function */
  );
  
  if (port_class == NULL) {
    OPENER_TRACE_ERR("Failed to create Port Object class\n");
    return kEipStatusError;
  }
  
  /* Initialize port name and type name strings */
  SetCipShortStringByCstr(&g_port[0].port_name, "EtherNet/IP Port");
  SetCipShortStringByCstr(&g_port[0].port_type_name, "TCP");
  
  /* Get instance #1 */
  CipInstance *instance = GetCipInstance(port_class, 1);
  if (instance == NULL) {
    OPENER_TRACE_ERR("Failed to get Port instance 1\n");
    return kEipStatusError;
  }
  
  /* Register attributes */
  InsertAttribute(instance,
                  1,
                  kCipUsint,
                  EncodeCipUsint,
                  NULL,
                  &g_port[0].port_type,
                  kGetableSingleAndAll);
  
  InsertAttribute(instance,
                  2,
                  kCipUsint,
                  EncodeCipUsint,
                  NULL,
                  &g_port[0].port_number,
                  kGetableSingleAndAll);
  
  InsertAttribute(instance,
                  3,
                  kCipEpath,
                  EncodeCipEPath,
                  NULL,
                  &g_port[0].link_path,
                  kGetableSingleAndAll);
  
  InsertAttribute(instance,
                  4,
                  kCipShortString,
                  EncodeCipShortString,
                  NULL,
                  &g_port[0].port_name,
                  kGetableSingleAndAll);
  
  InsertAttribute(instance,
                  5,
                  kCipShortString,
                  EncodeCipShortString,
                  NULL,
                  &g_port[0].port_type_name,
                  kGetableSingleAndAll);
  
  InsertAttribute(instance,
                  6,
                  kCipUsint,
                  EncodeCipUsint,
                  NULL,
                  &g_port[0].node_address,
                  kGetableSingleAndAll);
  
  /* Register instance services */
  InsertService(port_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
  InsertService(port_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");
  
  OPENER_TRACE_INFO("Port Object initialized successfully\n");
  
  return kEipStatusOk;
}

