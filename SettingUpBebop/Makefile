EXEC_NAME=BebopDroneDecodeStream #output filename

SDK_DIR=../../../ARSDKBuildUtils
IDIR=./

CC=gcc
#CXX=g++
CFLAGS=-I$(IDIR) -I$(SDK_DIR)/Targets/Unix/Install/include -g


OBJDIR=obj
LDIR = $(SDK_DIR)/Targets/Unix/Install/lib

EXTERNAL_LIB=-lncurses -lavcodec -lavformat -lswscale -lavutil -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lopencv_hal -lm

LIBS=-L$(LDIR)  -larsal -larcommands -larnetwork -larnetworkal -lardiscovery -larstream $(EXTERNAL_LIB)
LIBS_DBG=-L$(LDIR)  -larsal_dbg -larcommands_dbg -larnetwork_dbg -larnetworkal_dbg -lardiscovery_dbg -larstream_dbg $(EXTERNAL_LIB)

_DEPS = BebopDroneDecodeStream.h DecoderManager.h ihm.h yuv.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = BebopDroneDecodeStream.o DecoderManager.o ihm.o yuv.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ)) 


FIFO = video_decoded_fifo
FIFORGB = video_decoded_fifo_RGB

CSOURCES = BebopDroneDecodeStream.c DecoderManager.c ihm.c yuv.c

all: $(EXEC_NAME)
	@[ -p $(FIFO) ] || mkfifo $(FIFO)
	@[ -p $(FIFORGB) ] || mkfifo $(FIFORGB)
	
	

$(OBJDIR)/%.o: %.c $(DEPS)
	@ [ -d $(OBJDIR) ] || mkdir $(OBJDIR)
	@ $(CC) -c -o $@ $< $(CFLAGS) $(LIBS)
    

$(EXEC_NAME): $(OBJ)
	@ gcc -o $@ $^ $(CFLAGS) $(LIBS)


.PHONY: clean

clean:
	@ rm -f $(OBJDIR)/*.o *~ core $(INCDIR)/*~
	@ rm -rf $(OBJDIR)
	@ rm -f $(FIFO)
	@ rm -f $(FIFORGB)
	@ rm -f $(EXEC_NAME)

run:
	LD_LIBRARY_PATH=$(LDIR) ./$(EXEC_NAME)
