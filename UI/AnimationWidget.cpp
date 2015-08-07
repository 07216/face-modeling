#include "AnimationWidget.h"
#include "Util/SettingDialog.h"

AnimationWidget::AnimationWidget(FacePerformance* face, QWidget *parent) :
    TemplateWidget(parent)
{
    //timer = new QTimer();
    //itime = 0;
    //timer->setInterval(5000);
    //timer->start();
    ktimer=0;
    FacePerformanceFunction::LoadRegisterIndexFromFile(recognitionIndex,
                                                       ConfigParameter::face_recognition_selection_index);
    this->face = face;

    this->setAutoFillBackground (true);
    QPalette palette;
    palette.setColor(QPalette::Active, static_cast<QPalette::ColorRole>(10), QColor(96,98,90));
    this->setPalette(palette);
    number_face = FacePerformanceFunction::GetNumFace();

    initLeft();
    initRight();
    initBottom();
    connect(face, SIGNAL(ShowColorImage(uchar*,int,int)), this, SLOT(ShowColorImage(uchar*,int,int)));

    //connect(timer,SIGNAL(timeout()),this,SLOT(onTimerOut()));
}
/*
void AnimationWidget::onTimerOut()
{
    //itime++;
    int num_face = FacePerformanceFunction::GetNumFace();
    if(num_face != 0)
    {
        idget0->changeid((itime+0)%num_face);
        idget1->changeid((itime+1)%num_face);
        idget2->changeid((itime+2)%num_face);
        idget3->changeid((itime+3)%num_face);
        idget4->changeid((itime+4)%num_face);
        idget5->changeid((itime+5)%num_face);
        idget6->changeid((itime+6)%num_face);
        idget7->changeid((itime+7)%num_face);
    }

}
*/
void AnimationWidget::initLeft()
{
    configButton = new QPushButton(tr("&Config"));
    startButton = new QPushButton(tr("Start"));
    changeButton = new QPushButton(tr("Change user"));
    recognitionButton = new QPushButton(tr("Recognition"));
    saveButton = new QPushButton(tr("Add to database"));
    userIdText = new QLineEdit();
    userNameText = new QLineEdit;

    configButton->setEnabled(true);
    startButton->setEnabled(true);
    changeButton->setEnabled(false);
    recognitionButton->setEnabled(false);
    saveButton->setEnabled(false);
    userIdText->setEnabled(false);
    userNameText->setEnabled(true);
    int num_face = FacePerformanceFunction::GetNumFace();
    userIdText->setText(QString("id: ").append(QString::number(num_face)));
    userNameText->setText("name: ");

    SettingDialog * config = new SettingDialog;
    QGridLayout * layout = new QGridLayout;

    layout->addWidget(configButton,0,0);
    layout->addWidget(startButton,1,0);
    layout->addWidget(changeButton, 2, 0);
    layout->addWidget(recognitionButton, 3, 0);
    layout->addWidget(saveButton, 4, 0);
    layout->addWidget(userIdText, 5, 0);
    layout->addWidget(userNameText, 6, 0);
    configWidget->setLayout(layout);

    connect(configButton, SIGNAL(clicked()), config, SLOT(exec()));
    connect(config, SIGNAL(update()), face, SLOT(UpdateParas()));
    connect(startButton, SIGNAL(clicked()), this, SLOT(startSlot()));
    connect(startButton, SIGNAL(clicked()), face, SLOT(StartTrackThread()));

    connect(changeButton, SIGNAL(clicked()), this, SLOT(changeUserSlot()));
    connect(recognitionButton, SIGNAL(clicked()), this, SLOT(recognitionSlot()));
    connect(saveButton, SIGNAL(clicked()), this, SLOT(saveSlot()));
#ifdef RECOGNITION
    recognitionButton->setVisible(true);
#else
    recognitionButton->setVisible(false);
#endif
    //saveButton->setVisible(false);
}

void AnimationWidget::initRight()
{
    QHBoxLayout * showLayout = new QHBoxLayout;
    vlayout = new QVBoxLayout;
    spl=new QSplitter(Qt::Vertical);
    showLayout->addWidget(face->userImage);
    showLayout->addWidget(face->userViewMesh);
    //showLayout->addWidget(face->avatarViewMesh);
    //showLayout->addWidget(idget);
    //idget->setVisible(false);
    image0=new QLabel;
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    cv::Mat cv_img = cv::imread(dir_face_database + "ladeng.jpg");
    cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
    QImage qimage(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
    QFont font("宋体", 45, QFont::Bold, true);
    QPainter qp(&qimage);
    qp.setPen(Qt::red);
    qp.setFont(font);
    QRectF rect(10, 140, 350, 350);
    qp.drawText(rect, QString("terrorist!"));
    image0->setPixmap(QPixmap::fromImage(qimage.scaled(qimage.size(), Qt::KeepAspectRatio)));
    meshwidget0=new idwidget;
    vlayout->addWidget(image0);
    vlayout->addWidget(meshwidget0);
    vlayout->setAlignment(image0,Qt::AlignCenter);
    vlayout->setStretch(0,1);
    vlayout->setStretch(1,5);
    showLayout->addLayout(vlayout);

    image0->setVisible(false);
    meshwidget0->setVisible(false);
    //showLayout->setSpacing(0);
    //showLayout->setContentsMargins(0, 0, 0, 0);
    showLayout->setStretchFactor(face->userImage,1);
    showLayout->setStretchFactor(face->userViewMesh,1);
    showLayout->setStretchFactor(vlayout,1);
    showAllWidget->setLayout(showLayout);
}

void AnimationWidget::initBottom()
{
    preButton = new QPushButton(tr("&Previous\nPage"));
    nextButton = new QPushButton(tr("&Next\nPage"));
    preButton->setEnabled(true);
    nextButton->setEnabled(true);
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    images.clear();
    for(int i=0;i<4;i++)
    {
        QLabel* tmp=new QLabel;
        std::string filename = dir_face_database + QString::number(i).toStdString() + ".jpg";
        cv::Mat cv_img = cv::imread(filename);
        QImage image(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        //QImage image(filename.c_str());
        QPainter qp(&image);
        qp.setPen(Qt::blue);
        QFont font("宋体", 40, QFont::Bold, true);
        qp.setFont(font);
        //设置一个矩形
        QRectF rect1(10, 10, 300, 300);
        //为了更直观地看到字体的位置，我们绘制出这个矩形
        //qp.drawRect(rect);
        qp.setPen(QColor(Qt::red));
        //我们这里先让字体水平居中
        qp.drawText(rect1, QString("id: ").append( QString::number(i)));
        tmp->setPixmap(QPixmap::fromImage(image.scaled(image.size()/3, Qt::KeepAspectRatio)));
        images.push_back(tmp);
    }
/*
    idwidgets.clear();
    for(int i=0;i<number_face;i++)
    {
        idwidget* tmp =new idwidget;
        tmp->changeid(i);
        idwidgets.push_back(tmp);
    }
*/
    //meshwidgets.clear();
    //meshwidgets.push_back(meshwidget1);
   // meshwidgets.push_back(meshwidget2);
    //meshwidgets.push_back(meshwidget3);
    //meshwidgets.push_back(meshwidget4);
    meshwidget1=new idwidget;
    meshwidget2=new idwidget;
    meshwidget3=new idwidget;
    meshwidget4=new idwidget;
    meshwidget1->changeid(0);
    meshwidget2->changeid(1);
    meshwidget3->changeid(2);
    meshwidget4->changeid(3);
    QHBoxLayout * idLayout =new QHBoxLayout;
    QHBoxLayout * idLayout1 =new QHBoxLayout;
    QHBoxLayout * idLayout2 =new QHBoxLayout;
    idsp =new QHBoxLayout();
    QSplitter * idsp1 =new QSplitter();

    idsp->addWidget(preButton);
    idsp->addWidget(images[0]);
    idsp->addWidget(meshwidget1);
    idsp->addWidget(images[1]);
    idsp->addWidget(meshwidget2);
    idsp->addWidget(images[2]);
    idsp->addWidget(meshwidget3);
    idsp->addWidget(images[3]);
    idsp->addWidget(meshwidget4);
    idsp->addWidget(nextButton);

    connect(preButton, SIGNAL(clicked()), this, SLOT(ktimerreduce()));
    connect(nextButton, SIGNAL(clicked()), this, SLOT(ktimeradd()));

    //idLayout1->addWidget(idwidgets[4]);
    //idLayout1->addWidget(idwidgets[5]);
    //idLayout1->addWidget(idwidgets[6]);
    //idLayout1->addWidget(idwidgets[7]);\
    //idsp->setLayout(idLayout);
    //idsp1->setLayout(idLayout1);
    //idLayout->addWidget(idsp);
    //idLayout2->addWidget(idsp1);

    idWidget->setLayout(idsp);

}

void AnimationWidget::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Enter)
    {
        face->mutex_isInitFrame.lock();
        face->isInitFrame = true;
        face->mutex_isInitFrame.unlock();
        //face->avatarViewMesh->setVisible(true);
        image0->setVisible(false);
        meshwidget0->setVisible(false);
        face->userViewMesh->camera()->setZClippingCoefficient(50);
        //face->avatarViewMesh->camera()->setZClippingCoefficient(50);
        saveButton->setEnabled(true);
        recognitionButton->setEnabled(true);
    }
}

int AnimationWidget::recognitionSimilaryId(double& distance)
{
    int tmpi;
    double tmpd;
    int num_face = FacePerformanceFunction::GetNumFace();
    if (num_face == 0)
        return -1;

    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    Eigen::MatrixXd tar, tar_recognition;
    tar.resize(3*face->initParas.init_registerIndex_size, 1);
    tar = face->faceParas.neutral_expression.block(0, 0, 3*face->initParas.init_registerIndex_size, 1);
    tar.resize(3, face->initParas.init_registerIndex_size);
    FacePerformanceFunction::ChooseRecognitionFacePart(tar, tar_recognition, recognitionIndex);

    Eigen::VectorXd all_error(num_face);
    #pragma omp parallel for
    for (int i = 0; i < num_face; ++i)
    {
        std::string src_filename = dir_face_database + QString::number(i).toStdString() + ".bin";
        Eigen::MatrixXd src, src_recognition;
        FacePerformanceFunction::LoadInitRegisterPart_binary(src, src_filename.c_str());
        FacePerformanceFunction::ChooseRecognitionFacePart(src, src_recognition, recognitionIndex);
        Eigen::VectorXd T = (tar-src).rowwise().mean();
        src.colwise() += T;
        all_error(i) = (tar-src).colwise().norm().mean();
    }

    distance = all_error.minCoeff();
    distances = all_error;
    Eigen::VectorXi idss(num_face);
    for(int i = 0; i < num_face; i++)
    {
        idss(i) = i;
    }
    ids = idss;
    for (int i = 0; i < num_face; ++i)
    {
        for(int j = i; j < num_face; j++)
        {
            if (distances(i) > distances(j))
            {
                tmpd = distances(i);
                distances(i) = distances(j);
                distances(j) = tmpd;
                tmpi = ids(i);
                ids(i) = ids(j);
                ids(j) = tmpi;
            }
        }
    }
    std::cout<<"recognized error: "<<all_error.transpose()<<std::endl;
    if (distance > face->initParas.max_facerecognition_error)
        return -1;
    for (int i = 0; i < num_face; ++i)
    {
        if (all_error(i) <= distance)
            return i;
    }
}


void AnimationWidget::startSlot()
{
    //timer->stop();
    startButton->setEnabled(false);
    changeButton->setEnabled(true);
    recognitionButton->setEnabled(true);
    saveButton->setEnabled(true);
    face->mutex_isTracking.lock();
    face->isTracking = false;
    face->mutex_isTracking.unlock();
    face->mutex_isInitFrame.lock();
    face->isInitFrame = false;
    face->mutex_isInitFrame.unlock();
}

void AnimationWidget::ktimerreduce()
{
    ktimer-=4;
    if(ktimer<0)
    {
        ktimer=ktimer+number_face;
    }
    changeimage((ktimer)%number_face,0);
    changeimage((ktimer+1)%number_face,1);
    changeimage((ktimer+2)%number_face,2);
    changeimage((ktimer+3)%number_face,3);
    meshwidget1->changeid((ktimer)%number_face);
    meshwidget2->changeid((ktimer+1)%number_face);
    meshwidget3->changeid((ktimer+2)%number_face);
    meshwidget4->changeid((ktimer+3)%number_face);
    meshwidget1->id_mesh->updateGL();
    meshwidget2->id_mesh->updateGL();
    meshwidget3->id_mesh->updateGL();
    meshwidget4->id_mesh->updateGL();
}

void AnimationWidget::ktimeradd()
{
    ktimer += 4;

    changeimage((ktimer)%number_face,0);
    changeimage((ktimer+1)%number_face,1);
    changeimage((ktimer+2)%number_face,2);
    changeimage((ktimer+3)%number_face,3);
    meshwidget1->changeid((ktimer)%number_face);
    meshwidget2->changeid((ktimer+1)%number_face);
    meshwidget3->changeid((ktimer+2)%number_face);
    meshwidget4->changeid((ktimer+3)%number_face);
    meshwidget1->id_mesh->updateGL();
    meshwidget2->id_mesh->updateGL();
    meshwidget3->id_mesh->updateGL();
    meshwidget4->id_mesh->updateGL();
}

void AnimationWidget::changeUserSlot()
{
    face->mutex_isInitFrame.lock();
    face->isInitFrame = true;
    face->mutex_isInitFrame.unlock();

    //face->avatarViewMesh->setVisible(true);//false);
    image0->setVisible(false);
    meshwidget0->setVisible(false);
    face->userViewMesh->camera()->setZClippingCoefficient(50);
    //face->avatarViewMesh->camera()->setZClippingCoefficient(50);
    saveButton->setEnabled(true);
    recognitionButton->setEnabled(true);

    face->mutex_isTracking.lock();
    face->isTracking = true;
    face->mutex_isTracking.unlock();
    preButton->setEnabled(true);
    nextButton->setEnabled(true);
}

void AnimationWidget::recognitionSlot()
{
    //timer->stop();
    recognitionButton->setEnabled(false);
    changeButton->setEnabled(false);
    bool saveButtonState = saveButton->isEnabled();
    saveButton->setEnabled(false);

    //face->avatarViewMesh->setVisible(false);

    //recognition code here
    face->mutex_recognition.lock();
    double distance = 0;
    int bestId = recognitionSimilaryId(distance);

    showImagedis(ids(0),distances(0),0);
    showImagedis(ids(1),distances(1),1);
    showImagedis(ids(2),distances(2),2);
    showImagedis(ids(3),distances(3),3);
    meshwidget1->changeid(ids(0));
    meshwidget2->changeid(ids(1));
    meshwidget3->changeid(ids(2));
    meshwidget4->changeid(ids(3));

    //meshwidget0->id_mesh->updateGL();
    meshwidget1->id_mesh->updateGL();
    meshwidget2->id_mesh->updateGL();
    meshwidget3->id_mesh->updateGL();
    meshwidget4->id_mesh->updateGL();
    //meshwidget0->setVisible(true);
    image0->setVisible(true);
    meshwidget0->setVisible(true);
    showRecognizedImagescale(bestId,distance,3);
    face->mutex_recognition.unlock();

    changeButton->setEnabled(true);
    saveButton->setEnabled(saveButtonState);
    recognitionButton->setEnabled(true);
    preButton->setEnabled(false);
    nextButton->setEnabled(false);
}

void AnimationWidget::saveSlot()
{
    face->mutex_isTracking.lock();
    face->isTracking = false;
    face->mutex_isTracking.unlock();

    saveButton->setEnabled(false);
    recognitionButton->setEnabled(false);
    changeButton->setEnabled(false);

    face->mutex_paras.lock();
    face->mutex_allImage.lock();
    face->mutex_recognition.lock();

    int num_face = FacePerformanceFunction::GetNumFace();
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    //save all frame and clear
    /*{
        std::string filename = dir_face_database + QString::number(num_face).toStdString() + ".bin";
        int num_images = face->faceParas.allColorImage.size();
        FILE* fout = fopen(filename.c_str(), "wb");
        fwrite(&num_images, sizeof(int), 1, fout);
        for(int i=0; i<num_images; i++)
        {
            fwrite(face->faceParas.allColorImage[i].data, sizeof(uchar), 640*480*3, fout);
            fwrite(face->faceParas.allDepthImage[i].data, sizeof(unsigned short), 640*480*2, fout);
        }
        fclose(fout);
        face->faceParas.allColorImage.clear();
        face->faceParas.allDepthImage.clear();
    }*/
    //save blendshape
    {
        //neutral and delta_B
        {
            std::string filename = dir_face_database + QString::number(num_face).toStdString() + ".bs";
            FILE* fout = fopen(filename.c_str(), "wb");
            fwrite(face->faceParas.neutral_expression.data(),
                   sizeof(double),face->faceParas.neutral_expression.size(), fout);
            fwrite(face->faceParas.delta_B.data(),
                   sizeof(double), face->faceParas.delta_B.size(), fout);
            fclose(fout);
        }
        //save neutral to obj
        {
            //as .obj file
            std::string infilename = "Data/Template/template.obj";
            ifstream infile;
            infile.open(infilename);
            char c;
            while (infile>>c && c == 'v')
            {
                double x, y, z;
                infile>>x>>y>>z;
            }
            std::string filename = dir_face_database + QString::number(num_face).toStdString() + "_neutral.obj";
            FILE* fout = fopen(filename.c_str(), "w");
            double * vertices = face->faceParas.neutral_expression.data();
            for (int i = 0; i < face->faceParas.neutral_expression.size(); i+=3)
            {
                fprintf(fout, "v %lf %lf %lf\n", vertices[i], vertices[i+1], vertices[i+2]);
            }
            int v0, v1, v2, v3;
            infile>>v0>>v1>>v2>>v3;
            fprintf(fout, "f %d %d %d %d\n", v0, v1, v2, v3);
            while (infile>>c && c == 'f')
            {
                infile>>v0>>v1>>v2>>v3;
                fprintf(fout, "f %d %d %d %d\n", v0, v1, v2, v3);
            }
            fclose(fout);

            //save as .bin file
            std::string mesh_filename = dir_face_database + QString::number(num_face).toStdString() + ".bin";
            FacePerformanceFunction::SaveInitRegisterPart_binary(face->faceParas.neutral_expression, mesh_filename.c_str(), face->initParas.init_registerIndex_size);
        }
        //save picture
        {
            std::string img_filename = dir_face_database + QString::number(num_face).toStdString() + ".jpg";
            cv::imwrite(img_filename, face->faceParas.colorImage);
        }
    }
    //save username
    {
        std::string filename = dir_face_database + "id_name.txt";
        std::ofstream fout(filename.c_str(), std::ios::app);
        fout<<num_face<<'\t'<<userNameText->text().toStdString()<<std::endl;
        fout.close();
    }
    FacePerformanceFunction::SaveNumFace(num_face+1);
    userIdText->setText(QString::number(num_face+1));
    face->mutex_recognition.unlock();
    face->mutex_allImage.unlock();
    face->mutex_paras.unlock();
    changeButton->setEnabled(true);
    //number_face++;
    //idwidget * tmp =new idwidget;
    //tmp->changeid(number_face);
    //idwidgets.push_back(tmp);
    number_face++;
}

void AnimationWidget::showImagedis(int id, double distance,int index)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    if(id != -1)
    {
        std::string filename = dir_face_database + QString::number(id).toStdString() + ".jpg";
        cv::Mat cv_img = cv::imread(filename);
        QImage image(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        //QImage image(filename.c_str());
        QPainter qp(&image);
        qp.setPen(Qt::blue);
        QFont font("宋体", 40, QFont::Bold, true);
        qp.setFont(font);
        //设置一个矩形
         QRectF rect1(10, 10, 300, 300);
         QRectF rect2(10, 100, 300, 300);
        //为了更直观地看到字体的位置，我们绘制出这个矩形
        //qp.drawRect(rect);
        qp.setPen(QColor(Qt::red));
        //我们这里先让字体水平居中
        qp.drawText(rect1, QString("id: ").append( QString::number(id)));
        qp.drawText(rect2, QString("d: ").append( QString::number(distance, 'g', 3)));
        images[index]->setPixmap(QPixmap::fromImage(image.scaled(images[index]->size(), Qt::KeepAspectRatio)));
        //meshwidgets[index]=idwidgets[id];

    }
    else
    {
        cv::Mat cv_img = cv::imread(dir_face_database + "ladeng.jpg");
        cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
        QImage qimage(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        QFont font("宋体", 45, QFont::Bold, true);
        QPainter qp(&qimage);
        qp.setPen(Qt::red);
        qp.setFont(font);
        QRectF rect(10, 140, 350, 350);
        qp.drawText(rect, QString("terrorist!"));
        images[index]->setPixmap(QPixmap::fromImage(qimage.scaled(images[index]->size(), Qt::KeepAspectRatio)));
    }
}

void AnimationWidget::showRecognizedImage(int id, double distance)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    if(id != -1)
    {
        std::string filename = dir_face_database + QString::number(id).toStdString() + ".jpg";
        cv::Mat cv_img = cv::imread(filename);
        QImage image(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        //QImage image(filename.c_str());
        QPainter qp(&image);
        qp.setPen(Qt::blue);
        QFont font("宋体", 40, QFont::Bold, true);
        qp.setFont(font);
        //设置一个矩形
         QRectF rect1(10, 10, 300, 300);
         QRectF rect2(10, 100, 300, 300);
        //为了更直观地看到字体的位置，我们绘制出这个矩形
        //qp.drawRect(rect);
        qp.setPen(QColor(Qt::red));
        //我们这里先让字体水平居中
        qp.drawText(rect1, QString("id: ").append( QString::number(id)));
        qp.drawText(rect2, QString("d: ").append( QString::number(distance, 'g', 3)));
        image0->setPixmap(QPixmap::fromImage(image.scaled(image.size()/1.5, Qt::KeepAspectRatio)));
        //meshwidget0 = new idwidget;
        meshwidget0->changeid(id);
        meshwidget0->id_mesh->updateGL();
        meshwidget0->setVisible(true);
    }
    else
    {
        cv::Mat cv_img = cv::imread(dir_face_database + "ladeng.jpg");
        cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
        QImage qimage(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        QFont font("宋体", 45, QFont::Bold, true);
        QPainter qp(&qimage);
        qp.setPen(Qt::red);
        qp.setFont(font);
        QRectF rect(10, 140, 350, 350);
        qp.drawText(rect, QString("terrorist!"));
        image0->setPixmap(QPixmap::fromImage(qimage.scaled(qimage.size(), Qt::KeepAspectRatio)));
        meshwidget0->setVisible(false);
        //meshwidget0->close();
    }
}

void AnimationWidget::ShowColorImage(uchar *data, int cols, int rows)
{
    face->mutex_colorImage.lock();
    QImage image(data, cols, rows, QImage::Format_RGB888);
    face->userImage->setPixmap(QPixmap::fromImage(image.scaled(face->userImage->size(), Qt::KeepAspectRatio)));
    face->mutex_colorImage.unlock();
}

void AnimationWidget::changeimage(int id,int index)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    if(id != -1)
    {
        std::string filename = dir_face_database + QString::number(id).toStdString() + ".jpg";
        cv::Mat cv_img = cv::imread(filename);
        QImage image(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        //QImage image(filename.c_str());
        QPainter qp(&image);
        qp.setPen(Qt::blue);
        QFont font("宋体", 40, QFont::Bold, true);
        qp.setFont(font);
        //设置一个矩形
         QRectF rect1(10, 10, 300, 300);
         //QRectF rect2(10, 100, 300, 300);
        //为了更直观地看到字体的位置，我们绘制出这个矩形
        //qp.drawRect(rect);
        qp.setPen(QColor(Qt::red));
        //我们这里先让字体水平居中
        qp.drawText(rect1, QString("id: ").append( QString::number(id)));
        //qp.drawText(rect2, QString("d: ").append( QString::number(distance, 'g', 3)));
        images[index]->setPixmap(QPixmap::fromImage(image.scaled(image.size()/3, Qt::KeepAspectRatio)));
        //meshwidgets[index]=idwidgets[id];
    }
}

void AnimationWidget::showRecognizedImagescale(int id, double distance,double scale)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    if(id != -1)
    {
        std::string filename = dir_face_database + QString::number(id).toStdString() + ".jpg";
        cv::Mat cv_img = cv::imread(filename);
        QImage image(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        //QImage image(filename.c_str());
        QPainter qp(&image);
        qp.setPen(Qt::blue);
        QFont font("宋体", 40, QFont::Bold, true);
        qp.setFont(font);
        //设置一个矩形
         QRectF rect1(10, 10, 300, 300);
         QRectF rect2(10, 100, 300, 300);
        //为了更直观地看到字体的位置，我们绘制出这个矩形
        //qp.drawRect(rect);
        qp.setPen(QColor(Qt::red));
        //我们这里先让字体水平居中
        qp.drawText(rect1, QString("id: ").append( QString::number(id)));
        qp.drawText(rect2, QString("d: ").append( QString::number(distance, 'g', 3)));
        image0->setPixmap(QPixmap::fromImage(image.scaled(image.size()/scale, Qt::KeepAspectRatio)));
        //meshwidget0 = new idwidget;
        meshwidget0->changeid(id);
        meshwidget0->id_mesh->updateGL();
        meshwidget0->setVisible(true);
    }
    else
    {
        cv::Mat cv_img = cv::imread(dir_face_database + "ladeng.jpg");
        cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
        QImage qimage(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
        QFont font("宋体", 45, QFont::Bold, true);
        QPainter qp(&qimage);
        qp.setPen(Qt::red);
        qp.setFont(font);
        QRectF rect(10, 140, 350, 350);
        qp.drawText(rect, QString("terrorist!"));
        image0->setPixmap(QPixmap::fromImage(qimage.scaled(qimage.size(), Qt::KeepAspectRatio)));
        meshwidget0->setVisible(false);
        //meshwidget0->close();
    }
}
