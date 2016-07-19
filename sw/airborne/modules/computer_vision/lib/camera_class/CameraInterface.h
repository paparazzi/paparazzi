
//using namespace camera_interface;
class CameraInterface
{

public:
	CameraInterface();
	virtual ~CameraInterface();
	virtual void setSize(int width,int height)=0;

protected:
	int something;
};
