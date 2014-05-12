#
#  Copyright (c) 2014 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

SUBMAKE_OPTIONS=--no-builtin-rules --no-builtin-variables --no-print-directory

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------

all: clean test

test: test_usb_device

debug: debug_test_usb_device

clean: clean_test_usb_device

.PHONY: test_usb_device
test_usb_device:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making test_usb_device
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test_usb_device.mk test
	@echo ------------------------------------------------------------------------------------

.PHONY: clean
clean_test_usb_device:
	@echo ------------------------------------------------------------------------------------
	@echo --- Cleaning test_usb_device
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test_usb_device.mk clean
	@echo ------------------------------------------------------------------------------------

.PHONY: debug
debug_test_usb_device:
	@echo ------------------------------------------------------------------------------------
	@echo --- Debugging test_usb_device
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test_usb_device.mk debug
	@echo ------------------------------------------------------------------------------------
