
//using namespace camera_interface;
class CameraInterface
{

public:
	CameraInterface(){}
	~CameraInterface(){}
	virtual void setSize(int width,int height)=0;
	virtual void setColorGain(int green1, int green2, int red, int blue)=0;
	virtual void setCropLocation(int cropx, int cropy)=0;
};
