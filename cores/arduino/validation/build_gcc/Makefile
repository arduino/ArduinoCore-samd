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

test: test_arduino_zero test_samd21_xpro

clean: clean_arduino_zero clean_samd21_xpro

.PHONY: test_arduino_zero
test_arduino_zero:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making test_arduino_zero
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: clean_arduino_zero
clean_arduino_zero:
	@echo ------------------------------------------------------------------------------------
	@echo --- Cleaning test arduino_zero
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test.mk $@
	@echo ------------------------------------------------------------------------------------

.PHONY: debug_arduino_zero
debug_arduino_zero:
	@echo ------------------------------------------------------------------------------------
	@echo --- Debugging test arduino_zero
	@$(MAKE) DEBUG=1 VARIANT=arduino_zero $(SUBMAKE_OPTIONS) -f test.mk $@
	@echo ------------------------------------------------------------------------------------



.PHONY: test_samd21_xpro
test_samd21_xpro:
	@echo ------------------------------------------------------------------------------------
	@echo --- Making test_samd21_xpro
	@$(MAKE) DEBUG=1 VARIANT=samd21_xpro $(SUBMAKE_OPTIONS) -f test.mk
	@echo ------------------------------------------------------------------------------------

.PHONY: clean_samd21_xpro
clean:
	@echo ------------------------------------------------------------------------------------
	@echo --- Cleaning test samd21_xpro
	@$(MAKE) DEBUG=1 VARIANT=samd21_xpro $(SUBMAKE_OPTIONS) -f test.mk $@
	@echo ------------------------------------------------------------------------------------

.PHONY: debug_samd21_xpro
debug_samd21_xpro:
	@echo ------------------------------------------------------------------------------------
	@echo --- Debugging test samd21_xpro
	@$(MAKE) DEBUG=1 VARIANT=samd21_xpro $(SUBMAKE_OPTIONS) -f test.mk $@
	@echo ------------------------------------------------------------------------------------
