#include "Match_Instruments.hpp"

static  cv::Scalar getUniqueColorByIndex(size_t index, size_t totalPoints) {
    if (totalPoints == 0) return cv::Scalar(0, 0, 255);

    // OpenCV 中 Hue 的范围是 [0, 180]
    // 根据索引在色相环上均匀分布
    double hue = static_cast<double>(index) * 180.0 / static_cast<double>(totalPoints);

    // 创建一个 HSV 颜色：Hue动态，Saturation和Value设为最大(255)以保证颜色鲜艳
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    // 转换为 BGR 供绘图使用
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

    // 返回 Scalar(B, G, R)
    return cv::Scalar(bgr.data[0], bgr.data[1], bgr.data[2]);
}

InstrumentsMatcher::InstrumentsMatcher(std::string& templatePath):
    templateFileName(templatePath)
{

}

void InstrumentsMatcher::LoadTemplate()
{
    // 只有当 ID 列表发生变化时才重新加载
    if (pendingInstrumentsIDs != currtntInstrumentsIDs)
    {
        if (logFlag) spdlog::info(">>> Loading Instrument Templates...");
        Instruments.clear(); // 清空旧数据

        try {
            for (int idx : pendingInstrumentsIDs)
            {
                std::string filePath = templateFileName + "Instrument_" + std::to_string(idx) + ".yaml";
                if (logFlag) spdlog::info("Reading file: {}", filePath);

                YAML::Node config = YAML::LoadFile(filePath);
                InstrumentTemplate Template;

                // 1. 读取基础信息 
                if (config["instrument"]) {
                    YAML::Node info = config["instrument"];
                    Template.ID = info["id"].as<int>();
                    Template.name = info["name"].as<std::string>();

                    if (logFlag) {
                        spdlog::info("  [Instrument Info] ID: {}, Name: {}", Template.ID, Template.name);
                        if (info["name_en"]) spdlog::info("  Name(EN): {}", info["name_en"].as<std::string>());
                    }

                    // 2. 读取边长矩阵 (distance_matrix)
                    if (info["distance_matrix"] && info["distance_matrix"].IsSequence()) {
                        auto distMatrix = info["distance_matrix"];
                        if (logFlag) spdlog::info("  [Distance Matrix]:");
                        for (size_t i = 0; i < 3 && i < distMatrix.size(); ++i) {
                            double d_a = distMatrix[i][0].as<double>();
                            double d_b = distMatrix[i][1].as<double>();

                            Template.edges[i][0] = d_a;
                            Template.edges[i][1] = d_b;

                            if (logFlag) spdlog::info("    Row {}: [{:.2f}, {:.2f}]", i, d_a, d_b);
                        }
                    }
                }

                // 3. 读取点集信息并构建 vectorMap (及 matrixMap)
                if (config["points"] && config["points"].IsSequence()) {
                    if (logFlag) spdlog::info("  [Points Data]:");

                    for (const auto& pointNode : config["points"]) {
                        // 3.1 解析 encoding
                        std::bitset<3> pointKey;
                        std::string encodingStr = "UNKNOWN"; // 用于打印

                        if (pointNode["encoding"].IsSequence()) {
                            int b0 = pointNode["encoding"][0].as<int>();
                            int b1 = pointNode["encoding"][1].as<int>();
                            int b2 = pointNode["encoding"][2].as<int>();

                            pointKey[0] = b0;
                            pointKey[1] = b1;
                            pointKey[2] = b2;
                            encodingStr = fmt::format("[{},{},{}]", b0, b1, b2);
                        }

                        std::string pID = pointNode["id"] ? pointNode["id"].as<std::string>() : "Unknown";
                        if (logFlag) spdlog::info("    Point ID: {} (Encoding: {})", pID, encodingStr);

                        // 3.2 准备存放 3 个方向的向量和矩阵
                        std::array<cv::Point3f, 3> tempVectors;
                        std::array<cv::Mat, 3> tempMatrices;

                        // 初始化默认值
                        for (int k = 0; k < 3; k++) {
                            tempVectors[k] = cv::Point3f(0, 0, 0);
                            tempMatrices[k] = cv::Mat::eye(4, 4, CV_32F);
                        }

                        // 3.3 遍历该点的 edges 列表
                        if (pointNode["edges"].IsSequence()) {
                            for (const auto& edgeNode : pointNode["edges"]) {
                                int edgeIdx = edgeNode["encoding"].as<int>(); // 0, 1, or 2
                                std::string targetP = edgeNode["to"].as<std::string>();

                                // 读取 ref_vector (尖点向量)
                                bool hasVector = false;
                                if (edgeNode["ref_vector"].IsSequence()) {
                                    float x = edgeNode["ref_vector"][0].as<float>();
                                    float y = edgeNode["ref_vector"][1].as<float>();
                                    float z = edgeNode["ref_vector"][2].as<float>();
                                    tempVectors[edgeIdx] = cv::Point3f(x, y, z);
                                    hasVector = true;
                                }

                                if (logFlag) {
                                    std::string vecStr = hasVector ?
                                        fmt::format("({:.2f}, {:.2f}, {:.2f})", tempVectors[edgeIdx].x, tempVectors[edgeIdx].y, tempVectors[edgeIdx].z)
                                        : "NULL";

                                    spdlog::info("      -> To: {}, EdgeIdx: {}, RefVector: {}", targetP, edgeIdx, vecStr);
                                }
                            }
                        }

                        // 3.4 存入 Map
                        Template.vectorMap[pointKey] = tempVectors;
                        Template.matrixMap[pointKey] = tempMatrices;
                    }
                }

                // 4. 将解析好的 Template 加入列表
                Instruments.push_back(Template);
                if (logFlag) spdlog::info("  Successfully loaded Template ID: {}", Template.ID);
            }

            // 5. 更新当前 ID 列表，表示加载完成
            currtntInstrumentsIDs = pendingInstrumentsIDs;
            if (logFlag) spdlog::info("<<< All Templates Loaded.");

        }
        catch (const YAML::Exception& e) {
            std::cerr << "[Error] YAML Parsing Failed: " << e.what() << std::endl;
            if (logFlag) spdlog::error("[Error] YAML Parsing Failed: {}", e.what());
        }
    }
}


void InstrumentsMatcher::LoadTemplate(int& id)
{
    // 1. 构建文件路径
    std::string filePath = templateFileName + "Instrument_" + std::to_string(id) + ".yaml";

    try {
        // 2. 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(filePath);

        // 临时对象用于存储解析结果
        InstrumentTemplate tempTemplate;

        // 3. 解析基础信息 (已修正拼写为 "instrument")
        if (config["instrument"]) {
            YAML::Node info = config["instrument"];

            // 校验 YAML 中的 ID 是否与传入参数一致 (可选的安全检查)
            int yamlID = info["id"].as<int>();
            if (yamlID != id) {
                std::cerr << "[Warning] Loading ID " << id << " but YAML contains ID " << yamlID << std::endl;
            }
            tempTemplate.ID = yamlID;

            if (info["name"]) tempTemplate.name = info["name"].as<std::string>();

            // 4. 解析边长矩阵 distance_matrix
            if (info["distance_matrix"] && info["distance_matrix"].IsSequence()) {
                auto distMatrix = info["distance_matrix"];
                for (size_t i = 0; i < 3 && i < distMatrix.size(); ++i) {
                    if (distMatrix[i].IsSequence() && distMatrix[i].size() >= 2) {
                        tempTemplate.edges[i][0] = distMatrix[i][0].as<double>();
                        tempTemplate.edges[i][1] = distMatrix[i][1].as<double>();
                    }
                }
            }
        }

        // 5. 解析点集信息 (points)
        if (config["points"] && config["points"].IsSequence()) {
            for (const auto& pointNode : config["points"]) {

                // 5.1 解析 Encoding [x,x,x] -> bitset<3>
                std::bitset<3> pointKey;
                if (pointNode["encoding"].IsSequence()) {
                    // 假设 YAML [0,1,1] 对应 bitset 的 [0],[1],[2] 位
                    pointKey[0] = pointNode["encoding"][0].as<int>();
                    pointKey[1] = pointNode["encoding"][1].as<int>();
                    pointKey[2] = pointNode["encoding"][2].as<int>();
                }

                // 5.2 初始化向量和矩阵容器
                std::array<cv::Point3f, 3> tempVectors;
                std::array<cv::Mat, 3> tempMatrices;

                // 填充默认值 (0向量 和 单位矩阵)
                for (int k = 0; k < 3; k++) {
                    tempVectors[k] = cv::Point3f(0.f, 0.f, 0.f);
                    tempMatrices[k] = cv::Mat::eye(4, 4, CV_32F);
                }

                // 5.3 遍历该点的 edges，填充 ref_vector
                if (pointNode["edges"].IsSequence()) {
                    for (const auto& edgeNode : pointNode["edges"]) {
                        // encoding: 0, 1, or 2 (对应缺失边的索引)
                        int edgeIdx = edgeNode["encoding"].as<int>();

                        if (edgeIdx >= 0 && edgeIdx < 3) {
                            // 检查 ref_vector 是否有效 (非 null)
                            if (edgeNode["ref_vector"].IsSequence()) {
                                float x = edgeNode["ref_vector"][0].as<float>();
                                float y = edgeNode["ref_vector"][1].as<float>();
                                float z = edgeNode["ref_vector"][2].as<float>();
                                tempVectors[edgeIdx] = cv::Point3f(x, y, z);
                            }
                            // 如果需要读取矩阵，在此处添加逻辑
                        }
                    }
                }

                // 5.4 存入 Map
                tempTemplate.vectorMap[pointKey] = tempVectors;
                tempTemplate.matrixMap[pointKey] = tempMatrices;
            }
        }

        // 6. 更新类成员数据
        // 策略：检查是否已存在该ID，如果存在则覆盖，不存在则添加
        bool found = false;
        for (auto& instrument : Instruments) {
            if (instrument.ID == id) {
                found = true;
                break;
            }
        }
        if (!found) {
            Instruments.push_back(tempTemplate);
            // 同时更新 ID 列表记录
            currtntInstrumentsIDs.push_back(id);
        }

        std::cout << "[Info] Successfully loaded template for ID: " << id << std::endl;

    }
    catch (const YAML::Exception& e) {
        std::cerr << "[Error] Failed to load template " << filePath << ": " << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "[Error] Unknown exception: " << e.what() << std::endl;
    }
}

// 设定多个 ID
void InstrumentsMatcher::SetInstrumentsIDs(std::vector<int>& ids)
{
    // 直接将传入的 ID 列表赋值给 pending 列表
    // 这样下次调用 LoadTemplate() 时，会检测到 pending != current 从而触发重新加载
    pendingInstrumentsIDs = ids;
    LoadTemplate();
}

// 设定单个 ID
void InstrumentsMatcher::SetInstrumentsIDs(int id)
{
    // 清空当前的 pending 列表
    pendingInstrumentsIDs.clear();
    // 将新的单个 ID 加入
    pendingInstrumentsIDs.push_back(id);
    LoadTemplate(id);
}

void InstrumentsMatcher::GolbalMatch(std::vector<cv::Point3f>& Points, std::vector<bool> shadowIndex)
{
    if (matchCount > 2)  
    {  
        cv::Point3f NowPoint;
        int NowIndex;
        for (int index = 0; index < idForPoints.size(); index++)
        {
            if (idForPoints[index] == 0) { NowPoint = Points[index]; NowIndex = index; idForPoints[NowIndex] = -1; break; }
        }

        std::bitset<3> pointEncoding;
        int missEocoding;              //最终搜索结果 包括原点编码和缺失边索引
        int nowInstrumentIndex = 0;    //记录当前 算法一阶段 开始的搜索器械

        //搜索验证数据
        bool shaowIndexForPoint[3] = {0, 0, 0};  //记录 原点编码是否赋值
        std::vector<int> pointsOrder;  //记录 匹配到的点 在 编码里的索引，也就是点的顺序
        std::vector<int> pointsIndex;  //记录 匹配到的点的索引 //由于点会减少索引会改变

        //搜索结果
        std::array<cv::Point3f, 4> pointsToSave{}; //记录 最终的点 初始全置0
        pointsToSave[0] = NowPoint;

        AlgorithmStage nowStage = AlgorithmStage::FirstStage;
        for (int pointIndex = 1; pointIndex < Points.size(); pointIndex++)
        {
            if (nowStage == AlgorithmStage::ThirdStage) break; //匹配结束 退出循环
            if (idForPoints[pointIndex] != 0)   continue;
            double distance = cv::norm(Points[pointIndex] - NowPoint);
            switch (nowStage)
            {
                case AlgorithmStage::FirstStage:{
                    for (int instrumentIndex = nowInstrumentIndex; instrumentIndex < Instruments.size(); instrumentIndex++)  //nowInstrumentIndex 为了处理特殊情况 不加1
                    {
                        if (nowStage == AlgorithmStage::SecondStage) break; //跳出循环 以进入二阶段
                        if (shadowIndex[instrumentIndex] == true) continue; 
                        for (int firstIndex = 0; firstIndex < 3; firstIndex++)
                        {
                            if (nowStage == AlgorithmStage::SecondStage) break;
                            for (int secondIndex = 0; secondIndex < 2; secondIndex++)
                            {
                                double delta = std::abs(Instruments[instrumentIndex].edges[firstIndex][secondIndex] - distance);
                                if (delta < telorableError)
                                {
                                    pointEncoding[firstIndex] = secondIndex;
                                    shaowIndexForPoint[firstIndex] = 1;
                                    pointsIndex.push_back(pointIndex);
                                    pointsOrder.push_back(firstIndex);

                                    pointsToSave[firstIndex + 1] = Points[pointIndex];
                                    nowInstrumentIndex = instrumentIndex;
                                    nowStage = AlgorithmStage::SecondStage;
                                    break;
                                }
                            }
                        }
                    }
                    if (nowStage == AlgorithmStage::SecondStage) break;
                    else nowInstrumentIndex = 0;  
                    break;
                }

                case AlgorithmStage::SecondStage: {
                    for (int firstIndex = 0; firstIndex < 3; firstIndex++)
                    {
                        if (shaowIndexForPoint[firstIndex] == 1) continue;
                        for (int secondIndex = 0; secondIndex < 2; secondIndex++)
                        {
                            double delta = std::abs(Instruments[nowInstrumentIndex].edges[firstIndex][secondIndex] - distance);
                            if (delta < telorableError)
                            {
                                double checkDistance = cv::norm(Points[pointsIndex.back()] - Points[pointIndex]);
                                pointEncoding[firstIndex] = secondIndex;
                                shaowIndexForPoint[firstIndex] = 1;

                                int reMissEocoding = 3 - (firstIndex + pointsOrder.back()); //求解缺失索引，当前索引跟上次任一搜索到的索引。
                                bool checkEdgeEocoding = (pointEncoding[firstIndex] == pointEncoding[pointsOrder.back()]);
                                double checkDelta = std::abs(Instruments[nowInstrumentIndex].edges[reMissEocoding][checkEdgeEocoding] - checkDistance);
                                if (checkDelta < telorableError)
                                {
                                    pointsIndex.push_back(pointIndex);
                                    pointsOrder.push_back(firstIndex);
                                    pointsToSave[firstIndex + 1] = Points[pointIndex];
                                    pointEncoding[reMissEocoding] = !checkEdgeEocoding;
                                    missEocoding = reMissEocoding;
                                    shadowIndex[nowInstrumentIndex] = true;
                                    break;
                                }
                                else
                                {
                                    shaowIndexForPoint[firstIndex] = 0;
                                }
                            }
                        }
                    }
                    int count = shaowIndexForPoint[0] + shaowIndexForPoint[1] + shaowIndexForPoint[2];

                    if (count > 2) nowStage = AlgorithmStage::ThirdStage;
                    else if(pointIndex == Points.size() -1)
                    {
                        if (count > 1) { nowStage = AlgorithmStage::ThirdStage; break;};
                        pointIndex = pointsIndex[0]-1; //回到   进入算法二阶段之前的点 因为会进行自增，所以要先减一
                        nowInstrumentIndex += 1;
                        pointsIndex.clear();
                        pointsOrder.clear();
                        std::fill(std::begin(shaowIndexForPoint), std::end(shaowIndexForPoint), false); 
                        nowStage = AlgorithmStage::FirstStage;
                    }
                    break;
                }
            }
        }

        switch (nowStage)
        {
            case AlgorithmStage::FirstStage: // 杂点，而且与其他点较远 不构成边集中的相似边。
                //Points.erase(Points.begin());
                matchCount--;
                GolbalMatch(Points, shadowIndex);
                break;
            
            case AlgorithmStage::SecondStage: //杂点，跟末尾点组成了边集中相似的边。
                matchCount--;
                GolbalMatch(Points, shadowIndex);
                break;
            
            case AlgorithmStage::ThirdStage: 
                idForPoints[NowIndex] = nowInstrumentIndex +1;
                matchCount--;
                for (int idex : pointsIndex)
                {
                    idForPoints[idex] = nowInstrumentIndex + 1;
                    matchCount--;
                }
                Instruments[nowInstrumentIndex].AddPoins(pointsToSave);
                Instruments[nowInstrumentIndex].missingEocoding = missEocoding;
                Instruments[nowInstrumentIndex].pointEncoding = pointEncoding;
                Instruments[nowInstrumentIndex].IsFind = true;

                GolbalMatch(Points, shadowIndex);
                break;
        }

    }
    else return;
}

bool InstrumentsMatcher::Match(std::vector<cv::Point3f>& Points)
{
    idForPoints.assign(Points.size(), 0);
    matchCount = Points.size(); //待匹配点的个数

    std::vector<bool> shadowIndex;
    for (int i = 0; i < Instruments.size(); i++)
    {
        shadowIndex.push_back(false);
    }
    GolbalMatch(Points, shadowIndex);

    int count = 0;
    for (bool flag : shadowIndex)
    {
        count += flag;
    }
    return count;
}

void InstrumentsMatcher::Localize()
{
    for (InstrumentTemplate& instrument : Instruments)
    {
        if (instrument.IsFind == false) continue;
        else {
            instrument.IsFind = false;
            InstrumentPose pose;
            pose.ID = instrument.ID;

            std::array<cv::Point3f, 4> points;
            instrument.GetPoins(points);
            cv::Point3f x_vec = points[(instrument.missingEocoding +1)%3 +1] - points[0];
            double x_norm = cv::norm(x_vec);
            if (x_norm < 1e-6) continue; // 防止除零，三点重合直接跳过
            cv::Point3f x_axis = x_vec / x_norm;

            // 辅助向量: 
            cv::Point3f v02 = points[(instrument.missingEocoding + 2) % 3+1] - points[0];

            // Z轴: X 叉乘 V02 (得到垂直于 P0-P1-P2 平面的法向量)
            cv::Point3f z_vec = x_axis.cross(v02);
            double z_norm = cv::norm(z_vec);
            if (z_norm < 1e-6) continue; // 防止三点共线
            cv::Point3f z_axis = z_vec / z_norm;

            // Y轴: Z 叉乘 X (右手定则，自动垂直且为单位向量)
            cv::Point3f y_axis = z_axis.cross(x_axis);

            // --- 2. 填充变换矩阵 T ---

            cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

            // 使用 cv::Vec3f 强制转换，避免 cv::Mat(Point) 的构造陷阱
            // 旋转矩阵列分布：Col0=X, Col1=Y, Col2=Z
            cv::Mat(cv::Vec3f(x_axis)).copyTo(R.col(0).rowRange(0, 3)); // X轴 -> 第0列
            cv::Mat(cv::Vec3f(y_axis)).copyTo(R.col(1).rowRange(0, 3)); // Y轴 -> 第1列
            cv::Mat(cv::Vec3f(z_axis)).copyTo(R.col(2).rowRange(0, 3)); // Z轴 -> 第2列

            // 平移向量: P0 -> 第3列
            cv::Mat t = cv::Mat(cv::Vec3f(points[0]));

            cv::Mat pMat(cv::Vec3f(instrument.vectorMap.at(instrument.pointEncoding)[instrument.missingEocoding]));
            cv::Mat position = R* pMat + t;

            pose.Position = cv::Point3f(position);
            pose.R = R * instrument.matrixMap.at(instrument.pointEncoding)[instrument.missingEocoding];

        }
    }
}

bool InstrumentsMatcher::Calibrate(std::array<cv::Mat, 12>& rvec, std::array<cv::Mat, 12>& tvec)
{
    for (InstrumentTemplate& instrument : Instruments)
    {
        if (!instrument.IsFind.load()) continue;
        instrument.IsFind.store(false);
        std::array<cv::Point3f, 4> points{};
        instrument.GetPoins(points);

        for (const cv::Point3f& p : points)
        {
            if(p.x == 0.0f && p.y == 0.0f && p.z == 0.0f)   return false;
        }

        unsigned int key =
            (instrument.pointEncoding.test(0) ? 1u << 1 : 0) |  // 高位
            (instrument.pointEncoding.test(1) ? 1u << 0 : 0);   // 低位
        
        for (size_t i = 0; i < 4; i++)
        {
            switch (key%4)
            {
                case 0:
                    if (i != 0) std::swap_ranges(points.begin(), points.begin() + 2, points.begin() + 2);
                    for (int m = 0; m < 3; m++)
                    {
                        std::array<cv::Mat, 2> Frame = ComputeToolFrame(points, m);
                        rvec[key * 3 + m] = std::move(Frame[0]);
                        tvec[key * 3 + m] = std::move(Frame[1]);
                    }
                    key += 1;
                    break;
                case 1:
                    if (i != 0) {
                        std::swap(points[0], points[1]);
                        std::swap(points[2], points[3]);
                    }
                    for (int m = 0; m < 3; m++)
                    {
                        std::array<cv::Mat, 2> Frame = ComputeToolFrame(points, m);
                        rvec[key * 3 + m] = std::move(Frame[0]);
                        tvec[key * 3 + m] = std::move(Frame[1]);
                    }
                    key += 1;
                    break;
                case 2:
                    if (i != 0) {
                        std::swap(points[0], points[1]);
                        std::swap(points[2], points[3]);
                        std::swap_ranges(points.begin(), points.begin() + 2, points.begin() + 2);
                    }
                    for (int m = 0; m < 3; m++)
                    {
                        std::array<cv::Mat, 2> Frame = ComputeToolFrame(points, m);
                        rvec[key * 3 + m] = std::move(Frame[0]);
                        tvec[key * 3 + m] = std::move(Frame[1]);
                    }
                    key += 1;
                    break;
                case 3:
                    if (i != 0) {
                        std::swap(points[0], points[1]);
                        std::swap(points[2], points[3]);
                    }
                    for (int m = 0; m < 3; m++)
                    {
                        std::array<cv::Mat, 2> Frame = ComputeToolFrame(points, m);
                        rvec[key * 3 + m] = std::move(Frame[0]);
                        tvec[key * 3 + m] = std::move(Frame[1]);
                    }
                    key += 1;
                    break;
            }
        }
        return true;

    }

}

std::array<cv::Mat, 2> InstrumentsMatcher::ComputeToolFrame(const std::array<cv::Point3f, 4>& points,int misscoding)
{
    // misscoding 只能是 0/1/2，对应忽略点索引 1/2/3
    const int ignoreIdx = misscoding + 1;
    if (ignoreIdx < 1 || ignoreIdx > 3)
        throw std::invalid_argument("misscoding must be 0,1,2 (ignoreIdx must be 1..3)");

    // 选择参与构建的两个点索引：从 {1,2,3} 去掉 ignoreIdx
    int idxA = -1, idxB = -1;
    for (int i = 1; i <= 3; ++i) {
        if (i == ignoreIdx) continue;
        if (idxA == -1) idxA = i;
        else idxB = i;
    }
    // idxA < idxB 恒成立（按 i 从小到大挑）
    if (idxA == -1 || idxB == -1)
        throw std::runtime_error("Failed to select two points for frame construction");

    const cv::Point3f& p0 = points[0];
    const cv::Point3f& pA = points[idxA]; // 最小索引 -> X 轴方向
    const cv::Point3f& pB = points[idxB]; // 另一个点 -> 用于求 Z

    auto toVec3d = [](const cv::Point3f& p) -> cv::Vec3d {
        return cv::Vec3d(static_cast<double>(p.x),
            static_cast<double>(p.y),
            static_cast<double>(p.z));
        };

    const cv::Vec3d o = toVec3d(p0);
    cv::Vec3d vA = toVec3d(pA) - o; // 原点到A
    cv::Vec3d vB = toVec3d(pB) - o; // 原点到B

    const double eps = 1e-12;

    auto norm3 = [](const cv::Vec3d& v) -> double {
        return std::sqrt(v.dot(v));
        };

    auto normalize = [&](cv::Vec3d& v) {
        const double n = norm3(v);
        if (n < eps) throw std::runtime_error("Degenerate frame: zero-length direction vector");
        v /= n;
        };

    // X 轴
    cv::Vec3d x = vA;
    normalize(x);

    // 用另一向量与 X 叉乘得到 Z
    cv::Vec3d b = vB;
    normalize(b);

    cv::Vec3d z = x.cross(b);
    const double nz = norm3(z);
    if (nz < eps)
        throw std::runtime_error("Degenerate frame: points are (nearly) collinear, cannot define Z axis");
    z /= nz;

    // Y 轴保证正交右手系：Y = Z × X
    cv::Vec3d y = z.cross(x);
    const double ny = norm3(y);
    if (ny < eps)
        throw std::runtime_error("Degenerate frame: cannot define Y axis");
    y /= ny;

    // 组装旋转矩阵：列向量为 x, y, z
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
        x[0], y[0], z[0],
        x[1], y[1], z[1],
        x[2], y[2], z[2]
        );

    // 平移向量：原点 p0（3x1）
    cv::Mat t = (cv::Mat_<double>(3, 1) << o[0], o[1], o[2]);

    return { R, t };
}

void InstrumentsMatcher::ResetAllInstrumentFindFlags()
{
    for (auto& inst : Instruments)
    {
        inst.IsFind.store(false, std::memory_order_relaxed);
    }
}


void InstrumentsMatcher::Visuable(const cv::Mat& leftImage,
    const cv::Mat& rightImage,
    const std::vector<std::array<cv::Point2f, 2>>& map,
    BrowsableQueue& visualImageQueue)
{
    // 0. 安全检查
    if (leftImage.empty() || rightImage.empty()) return;
    if (map.size() != idForPoints.size()) {
        std::cerr << "Error: Points map size and ID vector size mismatch!" << std::endl;
        return;
    }

    int cols = leftImage.cols;
    int rows = leftImage.rows;
    int totalWidth = cols * 2;

    cv::Mat visualImage;
    // 1. 创建大画布
    visualImage.create(rows, totalWidth, CV_8UC3);
    cv::Mat leftROI = visualImage(cv::Rect(0, 0, cols, rows));
    cv::Mat rightROI = visualImage(cv::Rect(cols, 0, cols, rows));

    if (leftImage.channels() == 1) cv::cvtColor(leftImage, leftROI, cv::COLOR_GRAY2BGR);
    else leftImage.copyTo(leftROI);

    if (rightImage.channels() == 1) cv::cvtColor(rightImage, rightROI, cv::COLOR_GRAY2BGR);
    else rightImage.copyTo(rightROI);

    // 2. 遍历所有点对
    // 2. 遍历所有点对
    size_t numPoints = map.size();
    for (size_t i = 0; i < numPoints; ++i) {
        int id = idForPoints[i];

        // 获取坐标
        cv::Point2f ptLeft = map[i][0];
        cv::Point2f ptRightShifted = map[i][1] + cv::Point2f(cols, 0);

        // 生成颜色
        cv::Scalar color = getUniqueColorByIndex(i, numPoints);

        // ==========================================
        // 【关键修改】：调大参数
        // ==========================================
        int radius = 8;             // 半径：4 -> 8 (变大一倍)
        double fontScale = 1.2;     // 字体：0.5 -> 1.2 (变大两倍多)
        int textThickness = 3;      // 字体粗细：2 -> 3 (加粗)
        cv::Point2f textOffset(12, -12); // 文字偏移：(5, -5) -> (12, -12) (离点远一点，别挡住点)

        // 白色描边颜色 (用于增强对比度)
        cv::Scalar white(255, 255, 255);

        // --- 绘制左图 ---

        // 1. 先画一个稍大的白色实心圆做底（形成描边效果）
        cv::circle(visualImage, ptLeft, radius + 2, white, -1);
        // 2. 再画彩色的实心圆
        cv::circle(visualImage, ptLeft, radius, color, -1);

        // 3. 画文字描边（先用白色画粗一点）
        cv::putText(visualImage, std::to_string(id), ptLeft + textOffset,
            cv::FONT_HERSHEY_SIMPLEX, fontScale, white, textThickness + 2);
        // 4. 画文字本体（再用彩色画细一点）
        cv::putText(visualImage, std::to_string(id), ptLeft + textOffset,
            cv::FONT_HERSHEY_SIMPLEX, fontScale, color, textThickness);


        // --- 绘制右图 (逻辑同上) ---

        cv::circle(visualImage, ptRightShifted, radius + 2, white, -1);
        cv::circle(visualImage, ptRightShifted, radius, color, -1);

        cv::putText(visualImage, std::to_string(id), ptRightShifted + textOffset,
            cv::FONT_HERSHEY_SIMPLEX, fontScale, white, textThickness + 2);
        cv::putText(visualImage, std::to_string(id), ptRightShifted + textOffset,
            cv::FONT_HERSHEY_SIMPLEX, fontScale, color, textThickness);
    }
    visualImageQueue.push(visualImage);
}

