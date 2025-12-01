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
#ifndef OPENER_CIPPORT_H_
#define OPENER_CIPPORT_H_

/** @file cipport.h
 * @brief Public interface of the Port Object (Class 0xF4)
 *
 * The Port Object represents a physical communication interface (port) on an
 * EtherNet/IP device. It provides port-level information and diagnostics.
 */

#include "typedefs.h"
#include "ciptypes.h"
#include "cipepath.h"

/** @brief Port Object class code */
#define CIP_PORT_CLASS_CODE   0xF4U
static const CipUint kCipPortClassCode = CIP_PORT_CLASS_CODE;

/** @brief Port Type values */
typedef enum {
  kPortTypeReserved = 0x00,     /**< Reserved */
  kPortTypeTCP = 0x01,          /**< TCP/IP port */
  kPortTypeUDP = 0x02,         /**< UDP port */
  kPortTypeEtherNetIP = 0x03,   /**< EtherNet/IP port */
  kPortTypeDeviceNet = 0x04,    /**< DeviceNet port */
  kPortTypeControlNet = 0x05,   /**< ControlNet port */
  kPortTypeCompoNet = 0x06      /**< CompoNet port */
} CipPortType;

/** @brief Port Object instance data structure */
typedef struct {
  CipUsint port_type;           /**< Attribute #1: Port Type (e.g., TCP=1) */
  CipUsint port_number;         /**< Attribute #2: Port Number */
  CipEpath link_path;           /**< Attribute #3: Link Path to associated link object */
  CipShortString port_name;     /**< Attribute #4: Port Name (e.g., "EtherNet/IP Port") */
  CipShortString port_type_name; /**< Attribute #5: Port Type Name (e.g., "TCP") */
  CipUsint node_address;        /**< Attribute #6: Node Address (optional, 0 if not used) */
} CipPortObject;

/* public functions */
/** @brief Initialize the Port Object
 *
 *  @return kEipStatusOk if initialization was successful, otherwise kEipStatusError
 */
EipStatus CipPortInit(void);

/* global object instance(s) */
extern CipPortObject g_port[];

#endif /* OPENER_CIPPORT_H_ */

