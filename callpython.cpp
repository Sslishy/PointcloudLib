#include<python3.8/Python.h>
#include <numpy/arrayobject.h>
#include <opencv2/opencv.hpp>
 #include<python3.8/cpython/initconfig.h>
int main(int a, char** b)
{
    PyObject *pModule, *pDict, *pFRCNN, *pFrcnn;
    PyObject *result, *result1;
    cv::Mat sml_img = cv::imread("1.jpg");
    //初始化python
    Py_Initialize();
    import_array();
    if (!Py_IsInitialized())
    {
        printf("python初始化失败！");
        return 0;
    }
    //引入当前路径,否则下面模块不能正常导入
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('/home/slishy/Code/class/lib/build')");
    //引入py文件
    pModule = PyImport_ImportModule("show");
    assert(pModule != NULL);
    //获取模块字典，即整个py文件的对象
    pDict = PyModule_GetDict(pModule);
    assert(pDict != NULL);
    //通过字典属性获取模块中的类
    pFRCNN = PyDict_GetItemString(pDict, "cv");
    // pFRCNN = PyDict_GetItemString(pDict, "Faster_RCNN");
    assert(pFRCNN != NULL);
    // 实例化faster-rcnn检测器
    pFrcnn = PyObject_CallObject(pFRCNN, nullptr);
    assert(pFrcnn != NULL);
     

    //分配参数列表的个数
     PyObject *pArgs = PyTuple_New(1);
    if (!sml_img.isContinuous()) { sml_img = sml_img.clone(); }
    //建立一个numpy的数据格式 长 宽和通道数
    npy_intp dims[] = {sml_img.rows, sml_img.cols, 3};

    //将图片转成numpy
    PyObject *pValue = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, sml_img.data);
    //将参数转换到参数列表当中
    PyTuple_SetItem(pArgs, 0, pValue);  /* pValue的引用计数被偷偷减一，无需手动再减 */
    // PyTuple_SetItem(pArgs, 0, NULL);
    //PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", 2));    /* 图像放大2倍 */


    /* 调用函数 */
    PyObject *pFunc = PyObject_GetAttrString(pFrcnn, "read");
     assert(pFunc != NULL);
     PyRun_SimpleString("print('-'*10, 'Python start', '-'*10)");
      //PyObject_CallMethod(pFunc, "read",NULL);
    // 调用方法 传入参数列表
    PyObject *pRetValue = PyObject_CallObject(pFunc,pArgs);
     assert(pRetValue != NULL);
    int data;
   /* 解析返回结果 */
    PyArrayObject *ret_array;
    // 将返回结果转成数组
    PyArray_OutputConverter(pRetValue, &ret_array);
    //取数组的shape
    npy_intp *shape = PyArray_SHAPE(ret_array);
     for(npy_intp row = 0; row < shape[0]; row++)
    {
    	for(npy_intp col = 0; col < shape[1]; col++)
    	{
			data = *(int*)PyArray_GETPTR2(ret_array,row,col);
			std::cout << data <<std::endl;
		}
		
    }

    
    // 将数组转成图片
    cv::Mat big_img(shape[0], shape[1], CV_8UC3, PyArray_DATA(ret_array));
    std::cout << big_img <<std::endl;

   // PyObject_CallMethod(pFrcnn, "read", "f", pValue);
    // 调用类的方法
    /*result = PyObject_CallMethod(pFrcnn, "get_msg", "");
    
    PyObject_CallMethod(pFrcnn, "set_msg", "si", "tylssss", 24);
    result1 = PyObject_CallMethod(pFrcnn, "get_msg", "");
    // 创建参数
    // PyObject *pArgs = PyTuple_New(2); //函数调用的参数传递均是以元组的形式打包的,2表示参数个数
 
    //输出返回值
    char* name;
    int age;
    PyArg_ParseTuple(result, "si", &name, &age);
    printf("%s-%d\n", name, age);
    PyArg_ParseTuple(result1, "si", &name, &age);
    printf("%s-%d\n", name, age);
 
    PyRun_SimpleString("print('-'*10, 'Python end', '-'*10)");*/
 
    //释放python
    Py_DECREF(pFrcnn);
    Py_Finalize();
}