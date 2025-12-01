/*******************************************************************************
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
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
#ifndef OPENER_CIPPARAMETER_H_
#define OPENER_CIPPARAMETER_H_

#include "typedefs.h"
#include "ciptypes.h"

/** @brief Parameter class code */
#define kCipParameterClassCode 0x0FU

/** @brief Parameter Object instance structure
 * 
 * This structure is used internally by the Parameter Object implementation.
 */
/* Suppress unused typedef warning - this typedef is part of the API but may not be used in all translation units */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
typedef struct {
  CipShortString parameter_name;      /**< Attribute 1: Parameter Name */
  CipUsint data_type;                 /**< Attribute 2: Data Type (internal) */
  void *parameter_value;              /**< Attribute 2: Parameter Value (pointer to actual value) */
  CipShortString units;               /**< Attribute 3: Parameter Units */
  CipShortString help_string;         /**< Attribute 4: Help String */
  void *minimum_value;                /**< Attribute 5: Minimum Value (pointer) */
  void *maximum_value;                /**< Attribute 6: Maximum Value (pointer) */
  void *default_value;                /**< Attribute 7: Default Value (pointer) */
  CipUsint data_type_code;            /**< Attribute 8: Data Type Code (CIP data type) */
  CipUint parameter_id;               /**< Internal: Parameter instance ID */
  CipAttributeEncodeInMessage encode_func;      /**< Internal: Encode function for this data type */
  CipAttributeDecodeFromMessage decode_func;       /**< Internal: Decode function for this data type */
} CipParameterInstance;
#pragma GCC diagnostic pop

/* public functions */
/** @brief CIP Parameter object constructor
 *
 * @returns kEipStatusError if the class could not be created, otherwise kEipStatusOk
 */
EipStatus CipParameterInit(void);

/** @brief Get ACD timeout value from Parameter Object Instance #21
 *
 * @returns ACD timeout in seconds (1-3600), or 10 if not set or invalid
 * Note: Returns uint16_t to avoid header dependencies in lwIP code
 */
uint16_t CipParameterGetAcdTimeout(void);

#endif /* OPENER_CIPPARAMETER_H_ */

