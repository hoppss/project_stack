#include <cmath>

const double pi = 3.1415926535897932384626;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;
const double x_pi = 3.14159265358979324 * 3000.0 / 180.0;

double transformLat(double x,double y)
{
	double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 *sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
	ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
	return ret;
}

double transformLon(double x,double y)
{
	double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
	ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
	ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
	ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
	return ret;
}

bool out_of_china(double lng,double lat) 
{
	// γ��3.86~53.55,����73.66~135.05 
	return !(lng > 73.66 && lng < 135.05 && lat > 3.86 && lat < 53.55);
}

/**
* WGS84ת����GCj02
* IN @param lng
* IN @param lat
* OUT @param mglng
* OUT @param mglat
* @returns void
*/
void wgs84togcj02(double lng,double lat ,double &mglng,double &mglat) 
{
    if (out_of_china(lng, lat)) {
		mglng = lng;
		mglat = lat;
		return;
    } 
	else 
	{
      double dlat = transformLat(lng - 105.0, lat - 35.0);
      double dlng = transformLon(lng - 105.0, lat - 35.0);
      double radlat = lat / 180.0 * pi;
      double magic = sin(radlat);
      magic = 1 - ee * magic * magic;
      double sqrtmagic = sqrt(magic);
      dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
      dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);
      mglat = lat + dlat;
      mglng = lng + dlng;
      cout<<"mglat: "<<mglat<<"  mglng: "<<mglng<<endl;
      return;
  }
}

/**
* ����GCJ02 ת��Ϊ WGS84
* IN @param lng
* IN @param lat
* OUT @param wglng
* OUT @param wglat
* @returns void
*/
void gcj02towgs84(double lng,double lat ,double &wglng,double &wglat) 
{
    if (out_of_china(lng, lat)) {
		wglng = lng;
		wglat = lat;
		return;
    } 
    else {
      double dlat = transformLat(lng - 105.0, lat - 35.0);
      double dlng = transformLon(lng - 105.0, lat - 35.0);
      double radlat = lat / 180.0 * pi;
      double magic = sin(radlat);
      magic = 1 - ee * magic * magic;
      double sqrtmagic = sqrt(magic);
      dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi);
      dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * pi);
      double mglat = lat + dlat;
      double mglng = lng + dlng;
	  wglat = lat * 2 - mglat;
	  wglng = lng * 2 - mglng;
      return;
    }
}