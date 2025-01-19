/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: edoras_app_msg.h
**
** Purpose:
**  Define Edoras App Messages and info
**
** Notes:
**
**
*******************************************************************************/
#ifndef _edoras_app_msg_h_
#define _edoras_app_msg_h_

/**
 * EdorasApp command codes
 */
#define EDORAS_APP_NOOP_CC        0
#define EDORAS_APP_SET_TWIST_CC   1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct 
{
   CFE_MSG_CommandHeader_t CmdHeader;
} EdorasAppNoArgsCmd_t;


/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef EdorasAppNoArgsCmd_t EdorasAppNoopCmd_t;
//typedef EdorasAppTwistCmd_t  EdorasAppTwistStateCmd_t;

/*************************************************************************/


#endif /* _edoras_app_msg_h_ */

/************************/
/*  End of File Comment */
/************************/



















