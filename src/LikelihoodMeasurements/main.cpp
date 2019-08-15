#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "MeanValDistance.h"
#include "SqrdIntensityDiff.h"
#include "HistDiff.h"
#include <sstream>
#include <list>

/*Test File for the Likelihood measurements implemented*/

using namespace airMCL;

/*Simple object to store an Id and a weight associated to that ID*/
class weightedID{

public:

    /*Default constructor*/
    weightedID(): id(0), weight(0.0){ }

    /*Constructor*/
    weightedID(int ident, double w){
        id = ident;
        weight = w;
    }

    /*Print the object to std output*/
    void print(){
        std::cout << "ID: " << id << " Weight: " << weight << std::endl;
    }

    int id;        //The Id
    double weight; //The weight associated to the ID
};

/* "<" operator. Needed to sort a list of weightedID's*/
bool operator<(weightedID w1, weightedID w2){
    return w1.weight<w2.weight;
}

int main() {

    int baseImage = 0; //Id of the base image used for comparison
    int nImages = 100;
    double totalW; //Sum of all weights
    double sim; //Current similatiry measure

    std::list<weightedID> wList; //A list of weights associated with an ID

    std::stringstream strbuf;

    cv::Mat images[nImages];//Image set

    /*Read images*/
    for(int i=0;i<nImages;i++){
        strbuf.str("");
        /*imageBasePath*/
        strbuf << "/home/robst/Dropbox/AirplaneMCL/imgSamples/sample";
        strbuf << i;
        strbuf << ".jpg";
        images[i]=cv::imread(strbuf.str().c_str());
    }

    /*Global pointer to Likelihood-implementing objects*/
    Likelihood *imgComp;

    /*Mean Value Distance*/
    MeanValDistance *meanComp = new MeanValDistance();

    /*Sum of Intensity Differece Squared*/
    SqrdIntensityDiff *sqrdIComp = new SqrdIntensityDiff();

    /*Color Histogram Difference*/
    HistDiff *histComp = new HistDiff();
    /*Configure the Histogram Difference Comparion Object*/
    histComp->setDiv(32);
    //Comparison methods: CV_COMP_INTERSECT, CV_COMP_CHISQR, CV_COMP_CORREL, CV_COMP_BHATTACHARYYA
    histComp->setComparisonType(CV_COMP_INTERSECT);

    /*CHOOSE ONE FROM THESE THREE LIKELIHOOD-IMPLEMENTING OBJECTS*/

    //imgComp = meanComp;
    //imgComp = sqrdIComp;
    imgComp = histComp;

    /*Compare images and store results*/
    totalW=0;
    for(int i=0;i<nImages;i++){
        sim = imgComp->imgMatch(images[baseImage], images[i]);
        totalW+=sim;
        //std::cout <<"Comparison of image "<< baseImage << " and image" <<
          //          i << ": " << sim << std::endl;
        wList.push_back(weightedID(i,sim));
    }

    /*An iterator to travel the list*/
    std::list<weightedID>::iterator it;

    /*Normalize Weights*/
    for (it=wList.begin(); it!=wList.end(); ++it)
        (*it).weight = (*it).weight/totalW;

    /*Sort List based on weights*/
    wList.sort();

    /*Print List*/
    for (it=wList.begin(); it!=wList.end(); ++it)
        (*it).print();

    // create image window named "My Image"
    //cv::namedWindow("My Image");
    // show the image on window
    //cv::imshow("My Image", image);
    // wait key for 5000 ms
    //cv::waitKey(5000);

    return 0;
}
