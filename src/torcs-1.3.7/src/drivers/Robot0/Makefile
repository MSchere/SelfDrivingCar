##############################################################################
#
#    file                 : Makefile
#    created              : jue mar 2 10:14:51 CET 2017
#    copyright            : (C) 2002 INVETT-Car
#
##############################################################################

ROBOT       = Robot0
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = $(wildcard *.cpp)

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml car5-trb1.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-Robot0_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-Robot0_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
