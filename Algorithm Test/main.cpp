#include "CV_Manager.hpp"
#include "Logger.hpp"


int main()
{
	InitLogger();

	CV_Manager manager;
	manager.LoadImagesSet(true);
	Sleep(10000); // 单位：毫秒
	std::vector<int> ids = { 1,2,3,4,5,6};
	manager.updateTemplate(ids);
	manager.VisualImageSet(true);
	// 循环 50 次
	for (int i = 0; i < 50; ++i)
	{
		// 打印当前进度，方便在控制台查看
		printf("=== Iteration: %d / 50 ===\n", i + 1);

		manager.FetchAndProcessImage();
		printf("1****\n"); // 注意：换行符是 \n

		manager.CorrespondenceAndReconstruction();
		printf("2****\n");

		manager.MatcherAndLocalize();
		printf("3****\n");

		printf("---------------------------\n");
	}


	system("pause");
	 
}
