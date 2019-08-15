
# Linea de compilacion en consola
g++ -std=c++0x -g -I/usr/include/opencv2 Distributions/Distributions.h LikelihoodMeasurements/Likelihood.h LikelihoodMeasurements/HistDiff.h LikelihoodMeasurements/HistDiff.cpp Models/Models.h Models/Models.cpp ParticleFilter/particle.h ParticleFilter/ParticleFilter.h ParticleFilter/ParticleFilter.cpp RobotFeatures/Observer.h RobotFeatures/RobotAirplane.cpp RobotFeatures/RobotAirplane.h RobotFeatures/State.h RobotFeatures/State.h RobotFeatures/main.cpp MeanShift/MeanShift.h MeanShift/MeanShift.cpp -lopencv_highgui -lopencv_legacy -lopencv_core -lopencv_imgproc -lopencv_video -fopenmp -o airMCL

CV_INCLUDE_PATH = -I/usr/include/opencv2

CV_LIBS         = -lopencv_highgui \
                  -lopencv_legacy \
                  -lopencv_core \
                  -lopencv_imgproc \
                  -lopencv_video 

LINK_OPENCV = $(CV_INCLUDE_PATH) $(CV_LIBS)

DIST_PATH = Distributions/Distributions.h

DRAW_PATH = Drawer/Drawer.h

HOOD_PATH = LikelihoodMeasurements/Likelihood.h \
            LikelihoodMeasurements/HistDiff.h \
            LikelihoodMeasurements/HistDiff.cpp
#            LikelihoodMeasurements/MeanValDistance.h \
#            LikelihoodMeasurements/MeanValDistance.cpp \
#            LikelihoodMeasurements/SqrIntensityDiff.h \
#            LikelihoodMeasurements/SqrIntensityDiff.cpp

MODELS_PATH = Models/Models.h \
              Models/Models.cpp

FILTER_PATH = Particle/particle.h \
              Particle/ParticleFilter.h \
              Particle/ParticleFilter.cpp


OBSERVER_PATH = RobotFeatures/Observer.h

AIRPLANE_PATH = RobotFeatures/RobotAirplane.cpp \
                RobotFeatures/RobotAirplane.h

STATE_PATH =  RobotFeatures/State.h


all:	airMCL

airMCL:	Distributions.o Drawer.o Likelihood.o HistDiff.o Models.o particle.o ParticleFilter.o Observer.o RobotAirplane.o State.o main.o
	g++ -g Distributions.o Drawer.o Likelihood.o HistDiff.o Models.o particle.o ParticleFilter.o Observer.o RobotAirplane.o State.o main.o $(LINK_OPENCV) -o airMCL

main.o:	RobotFeatures/main.cpp $(DRAW_PATH) $(FILTER_PATH) $(HOOD_PATH)
	g++ -g $(LINK_OPENCV) RobotFeatures/main.cpp

Drawer.o:	$(DRAW_PATH) $(FILTER_PATH) $(AIRPLANE_PATH)
	g++ -g $(LINK_OPENCV) Drawer/Drawer.h

ParticleFilter.o:	$(FILTER_PATH) $(AIRPLANE_PATH) $(MODELS_PATH) $(HOOD_PATH)
	g++ -g $(LINK_OPENCV) Particle/ParticleFilter.cpp

HistDiff.o:	$(HOOD_PATH) 
	g++ -g $(LINK_OPENCV) LikelihoodMeasurements/HistDiff.cpp

Distributions.o:	$(DIST_PATH)
	g++ -g $(LINK_OPENCV) Distributions/Distributions.h

Likelihood.o:	LikelihoodMeasurements/Likelihood.h
	g++ -g $(LINK_OPENCV) LikelihoodMeasurements/Likelihood.h

Models.o:	$(MODELS_PATH)
	g++ -g $(LINK_OPENCV) Models/Models.cpp

particle.o:	Particle/particle.h $(STATE_PATH)
	g++ -g $(LINK_OPENCV) Particle/particle.h

Observer.o:	$(OBSERVER_PATH)
	g++ -g $(LINK_OPENCV) RobotFeatures/Observer.h

RobotAirplane.o:	$(AIRPLANE_PATH) $(OBSERVER_PATH)
	g++ -g $(LINK_OPENCV) RobotFeatures/RobotAirplane.cpp

State.o:	$(STATE_PATH)
	g++ -g $(LINK_OPENCV) RobotFeatures/State.h


clean:
	/bin/rm airMCL	Distributions.o Drawer.o Likelihood.o HistDiff.o Models.o particle.o ParticleFilter.o Observer.o RobotAirplane.o State.o main.o

























