#instructions du makefile à https://riptutorial.com/makefile/example/21376/building-from-different-source-folders-to-different-target-folders

#n'utilisez que des .cpp et des .h

# Set project directory one level above of Makefile directory. $(CURDIR) is a GNU make variable containing the path to the current working directory
PROJDIR := $(realpath $(CURDIR)/..)
SOURCEDIR := $(PROJDIR)/Sources
BUILDDIR := $(PROJDIR)/Build

# Name of the final executable
TARGET = tutankhabot_executable

# Decide whether the commands will be shwon or not
VERBOSE = TRUE

#faire une liste de dirs pour chaque applications
MAINDIR = main_folder
LIDARSDKDIR = lidar_sdk/hal lidar_sdk/arch lidar_sdk/arch/linux lidar_sdk
EXAMPLEDIR = example
LIDARDIR = lidar
MAINCONTROLLERDIR = MainController
SPEEDCONTROLLERDIR = SpeedController
SPICOMDIR = SPICom 
CANCOMDIR = CanCom
FSMDIR = FSM
PATHDIR = path
# Create the list of directories
DIRS = $(MAINDIR) $(LIDARSDKDIR) $(EXAMPLEDIR) $(LIDARDIR) $(MAINCONTROLLERDIR) $(SPEEDCONTROLLERDIR) $(SPICOMDIR) $(CANCOMDIR) $(FSMDIR) $(PATHDIR) 
SOURCEDIRS = $(foreach dir, $(DIRS), $(addprefix $(SOURCEDIR)/, $(dir)))
TARGETDIRS = $(foreach dir, $(DIRS), $(addprefix $(BUILDDIR)/, $(dir)))

# Generate the GCC includes parameters by adding -I before each source folder
INCLUDES = $(foreach dir, $(SOURCEDIRS), $(addprefix -I, $(dir)))

# Add this list to VPATH, the place make will look for the source files
VPATH = $(SOURCEDIRS)

# Create a list of *.c sources in DIRS
SOURCES = $(foreach dir,$(SOURCEDIRS),$(wildcard $(dir)/*.cpp))

# Define objects for all sources
OBJS := $(subst $(SOURCEDIR),$(BUILDDIR),$(SOURCES:.cpp=.o))

# Define dependencies files for all objects
DEPS = $(OBJS:.o=.d)

# Name the compiler
CC = g++

#librairies à ajouter à la compilation
LIBS = -lpthread -lwiringPi


RM = rm -rf 
RMDIR = rm -rf 
MKDIR = mkdir -p
ERRIGNORE = 2>/dev/null
SEP=/


# Remove space after separator
PSEP = $(strip $(SEP))

# Hide or not the calls depending of VERBOSE
ifeq ($(VERBOSE),TRUE)
	HIDE =  
else
	HIDE = @
endif

# Define the function that will generate each rule
define generateRules
$(1)/%.o: %.cpp
	@echo Building $$@
	$(HIDE)$(CC) -c $$(INCLUDES) -o $$(subst /,$$(PSEP),$$@) $$(subst /,$$(PSEP),$$<) $(LIBS) -MMD
endef

.PHONY: all clean directories 

all: directories $(TARGET)

$(TARGET): $(OBJS)
	$(HIDE)echo Linking $@
	$(HIDE)$(CC) $(OBJS) -o $(TARGET) $(LIBS)

# Include dependencies
-include $(DEPS)

# Generate rules
$(foreach targetdir, $(TARGETDIRS), $(eval $(call generateRules, $(targetdir))))

directories: 
	$(HIDE)$(MKDIR) $(subst /,$(PSEP),$(TARGETDIRS)) $(ERRIGNORE)

# Remove all objects, dependencies and executable files generated during the build
clean:
	$(HIDE)$(RMDIR) $(subst /,$(PSEP),$(TARGETDIRS)) $(ERRIGNORE)
	$(HIDE)$(RM) $(TARGET) $(ERRIGNORE)
	@echo Cleaning done ! 