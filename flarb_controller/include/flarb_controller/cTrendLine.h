#include "flarb_mapbuilder/MapImage.h"


class cTrendLine
{
public:
	int Slope;
    int Intercept;
    int Start;
    int End	
private:
	int count;
    int xAxisValuesSum;
    int xxSum;
    int xySum;
    int yAxisValuesSum;
	int calculateSlope();
	int calculateIntercept();
	int calculateStart();
	int calculateEnd();
};


