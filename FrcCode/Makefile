##############################################################################
# Application specific info.  Specify the object target basename, object
# files to be created, and any precompiled libraries here.
##############################################################################
TARGET = FrcCode

OBJS = ifi_startup.o \
       ifi_utilities.o \
       main.o \
       user_camera.o \
       user_routines.o \
       user_routines_fast.o \
       user_SerialDrv.o

LIBS = FRC_library.lib

##############################################################################
# Compiler settings.  Specify compiler/linker flags here, as well as default
# directories for system headers/libraries.
##############################################################################
ifndef MCC18PATH
MCC18PATH = /home/WS/mcc18
endif
CFLAGS = -p=18F8520 -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa- -nw=2066
COPTS = -D_FRC_BOARD -D_USE_CMU_CAMERA
LDFLAGS = -m$(TARGET).map -o$(TARGET).cof
LIBDIRS = -L`cygpath -w $(MCC18PATH)/lib`
LINKER = 18f8520user.lkr
INCLUDEDIRS = -I`cygpath -w $(MCC18PATH)/include`
PATH += :$(MCC18PATH)/bin

##############################################################################
# Executable defines & rules - shouldn't need to change anything below here.
##############################################################################
CC  = mcc18
LD  = mplink
RM  = rm
PCC = cpp18

.SUFFIXES: .o .c .i

all: $(TARGET).hex

$(TARGET).hex: $(OBJS) $(LIBS)
	$(LD) $(LINKER) $(OBJS) $(LIBS) $(LDFLAGS) $(LIBDIRS)

.c.o:
	$(CC) $*.c -fo $@ $(INCLUDEDIRS) $(COPTS) $(CFLAGS)

.c.i:
	$(PCC) $*.c $(INCLUDEDIRS) > $@

clean:
	@echo "Cleaning build directory..."
	-@$(RM) *.o *.hex *.cof *.cod *.lst *.map *.err
