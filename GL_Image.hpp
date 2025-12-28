#ifdef _WIN32
#pragma comment(lib, "opengl32.lib")  // Windows下链接OpenGL系统库
#endif
#ifndef GL_IMAGE_HPP
#define GL_IMAGE_HPP

#include <QVector4D>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QImage>
#include <QColor>
#include <QSizePolicy>
#include <QResizeEvent>

class GLDualGrayImageWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit GLDualGrayImageWidget(QWidget* parent = nullptr)
        : QOpenGLWidget(parent),
        m_leftTexture(nullptr),
        m_rightTexture(nullptr),
        m_dividerColor(1.0f, 1.0f, 1.0f, 1.0f),
        m_dividerWidth(2),
        m_aspectRatio(3.0f / 1.0f),  // 总比例3:1（双图3:2并排 + 间距）
        m_spacing(20) {  // 图像间间距（可通过接口调整）
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setMinimumSize(1800, 600);  // 3:1比例基础尺寸
    }

    ~GLDualGrayImageWidget() override {
        delete m_leftTexture;
        delete m_rightTexture;
    }

    // 自定义分界线、间距
    void setDividerColor(const QColor& color) {
        m_dividerColor = QVector4D(color.redF(), color.greenF(), color.blueF(), color.alphaF());
        update();
    }

    void setDividerWidth(int width) {
        m_dividerWidth = qMax(1, width);
        update();
    }

    void setSpacing(int spacing) {
        m_spacing = qMax(0, spacing);
        update();
    }

public slots:
    void updateLeftImage(const QImage& grayImg) {
        m_leftImage = convertGrayToRGBA(grayImg);
        updateTexture(m_leftImage, &m_leftTexture);
        update();
    }

    void updateRightImage(const QImage& grayImg) {
        m_rightImage = convertGrayToRGBA(grayImg);
        updateTexture(m_rightImage, &m_rightTexture);
        update();
    }

    void updateBothImages(const QImage& leftGrayImg, const QImage& rightGrayImg) {
        m_leftImage = convertGrayToRGBA(leftGrayImg);
        m_rightImage = convertGrayToRGBA(rightGrayImg);
        updateTexture(m_leftImage, &m_leftTexture);
        updateTexture(m_rightImage, &m_rightTexture);
        update();
    }

protected:
    // 锁定窗口比例为3:1（含间距）
    void resizeEvent(QResizeEvent* event) override {
        QSize newSize = event->size();
        int optimalWidth = newSize.width();
        int optimalHeight = qRound(optimalWidth / m_aspectRatio);
        if (optimalHeight > newSize.height()) {
            optimalHeight = newSize.height();
            optimalWidth = qRound(optimalHeight * m_aspectRatio);
        }
        resize(optimalWidth, optimalHeight);
        QOpenGLWidget::resizeEvent(new QResizeEvent(size(), event->oldSize()));
    }

    void initializeGL() override {
        initializeOpenGLFunctions();
        glEnable(GL_TEXTURE_2D);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glEnable(GL_LINE_SMOOTH);
    }

    void resizeGL(int w, int h) override {
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, w, h, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        update();
    }

    void paintGL() override {
        glClear(GL_COLOR_BUFFER_BIT);

        // 计算有效绘制宽度（总宽 - 间距）
        int availableWidth = width() - m_spacing;
        int subBoxWidth = availableWidth / 2;
        int subBoxHeight = height();

        // 绘制左图（左半区，含间距前的区域）
        if (m_leftTexture && !m_leftImage.isNull()) {
            drawTexture(0, 0, subBoxWidth, subBoxHeight, m_leftTexture, m_leftImage);
        }

        // 绘制右图（右半区，间距后开始）
        if (m_rightTexture && !m_rightImage.isNull()) {
            drawTexture(subBoxWidth + m_spacing, 0, subBoxWidth, subBoxHeight, m_rightTexture, m_rightImage);
        }

        drawDivider();
    }

private:
    QImage m_leftImage;
    QImage m_rightImage;
    QOpenGLTexture* m_leftTexture;
    QOpenGLTexture* m_rightTexture;
    QVector4D m_dividerColor;
    int m_dividerWidth;
    const float m_aspectRatio;  // 总比例3:1（含间距）
    int m_spacing;  // 图像间间距（像素）

    QImage convertGrayToRGBA(const QImage& grayImg) {
        if (grayImg.isNull()) return QImage();
        if (grayImg.format() == QImage::Format_Grayscale8) {
            int w = grayImg.width(), h = grayImg.height();
            QImage rgbaImg(w, h, QImage::Format_RGBA8888);
            const uchar* grayData = grayImg.bits();
            uchar* rgbaData = rgbaImg.bits();
            int grayStride = grayImg.bytesPerLine();
            int rgbaStride = rgbaImg.bytesPerLine();

            for (int y = 0; y < h; ++y) {
                const uchar* grayRow = grayData + y * grayStride;
                uchar* rgbaRow = rgbaData + y * rgbaStride;
                for (int x = 0; x < w; ++x) {
                    uchar g = grayRow[x];
                    rgbaRow[x * 4] = g; rgbaRow[x * 4 + 1] = g; rgbaRow[x * 4 + 2] = g; rgbaRow[x * 4 + 3] = 255;
                }
            }
            return rgbaImg;
        }
        return grayImg.convertToFormat(QImage::Format_RGBA8888);
    }

    void updateTexture(const QImage& img, QOpenGLTexture** texture) {
        if (img.isNull()) {
            delete* texture;
            *texture = nullptr;
            return;
        }
        if (*texture) (*texture)->setData(img);
        else {
            *texture = new QOpenGLTexture(img, QOpenGLTexture::DontGenerateMipMaps);
            (*texture)->setMinificationFilter(QOpenGLTexture::Linear);
            (*texture)->setMagnificationFilter(QOpenGLTexture::Linear);
        }
    }

    void drawTexture(int x, int y, int w, int h, QOpenGLTexture* texture, const QImage& img) {
        if (!texture || img.isNull()) return;
        texture->bind();

        float imgW = img.width();   // 3072（3:2）
        float imgH = img.height();  // 2048
        float subBoxW = static_cast<float>(w);  // 子框宽（扣除间距后平分）
        float subBoxH = static_cast<float>(h);  // 子框高  
        // 因总比例锁定为3:1，子框比例仍为3:2，故qMin/qMax结果一致
        float scale = qMin(subBoxW / imgW, subBoxH / imgH);
        float drawW = imgW * scale;
        float drawH = imgH * scale;

        float offsetX = x + (subBoxW - drawW) / 2;
        float offsetY = y + (subBoxH - drawH) / 2;

        // 镜像翻转
        glBegin(GL_QUADS);
        glTexCoord2f(1, 0); glVertex2f(offsetX, offsetY);
        glTexCoord2f(0, 0); glVertex2f(offsetX + drawW, offsetY);
        glTexCoord2f(0, 1); glVertex2f(offsetX + drawW, offsetY + drawH);
        glTexCoord2f(1, 1); glVertex2f(offsetX, offsetY + drawH);
        glEnd();

        texture->release();
    }

    void drawDivider() {
        // 分界线居中于间距区域
        int dividerX = width() / 2;
        glLineWidth(static_cast<float>(m_dividerWidth));
        glColor4f(m_dividerColor.x(), m_dividerColor.y(), m_dividerColor.z(), m_dividerColor.w());
        glBegin(GL_LINES);
        glVertex2f(static_cast<float>(dividerX), 0.0f);
        glVertex2f(static_cast<float>(dividerX), static_cast<float>(height()));
        glEnd();
        glLineWidth(1.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
};

#endif // GL_IMAGE_HPP