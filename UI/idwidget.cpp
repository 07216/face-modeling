#include "idwidget.h"

idwidget::idwidget(QWidget *parent) :
    QWidget(parent)
{
    QHBoxLayout *main_layout=new QHBoxLayout();

    //QSplitter *splitter=new QSplitter(Qt::Vertical);
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
    //id_image=new QLabel;
    id_mesh=new idmesh;
    id_mesh->ReadMesh(dir_face_database+"111.obj");
    /*
    cv::Mat cv_img = cv::imread(dir_face_database + "ladeng.jpg");
    cv::cvtColor(cv_img, cv_img, CV_BGR2RGB);
    QImage qimage(cv_img.data, cv_img.cols, cv_img.rows, QImage::Format_RGB888);
    QFont font("宋体", 45, QFont::Bold, true);
    QPainter qp(&qimage);
    qp.setPen(Qt::red);
    qp.setFont(font);
    QRectF rect(10, 140, 350, 350);
    qp.drawText(rect, QString("terrorist!"));
    id_image->setPixmap(QPixmap::fromImage(qimage.scaled(qimage.size()/3, Qt::KeepAspectRatio)));
    splitter->addWidget(id_image);
    */
    //splitter->addWidget(id_mesh);
    main_layout->addWidget(id_mesh);
    setLayout(main_layout);
}

void idwidget::changeid(int id)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;

    PolyMesh tmpmesh;
    OpenMesh::IO::read_mesh(tmpmesh, dir_face_database + QString::number(id).toStdString() + "_neutral.obj");

    auto vv_it=tmpmesh.vertices_begin();
    for(auto v_it=id_mesh->mesh.vertices_begin();v_it!=id_mesh->mesh.vertices_end();v_it++)
    {
        id_mesh->mesh.point(v_it.handle()).data()[0]=tmpmesh.point(vv_it.handle()).data()[0];
        id_mesh->mesh.point(v_it.handle()).data()[1]=tmpmesh.point(vv_it.handle()).data()[1];
        id_mesh->mesh.point(v_it.handle()).data()[2]=tmpmesh.point(vv_it.handle()).data()[2];
        vv_it++;
    }
}

void idwidget::changeidd(int id,double distance)
{
    std::string dir_face_database = ConfigParameter::facedatabaseDir;
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
    id_image->setPixmap(QPixmap::fromImage(image.scaled(id_image->size(), Qt::KeepAspectRatio)));
    PolyMesh tmpmesh;
    OpenMesh::IO::read_mesh(tmpmesh,dir_face_database + QString::number(id).toStdString() + "_neutral.obj");
    auto vv_it=tmpmesh.vertices_begin();
    for(auto v_it=id_mesh->mesh.vertices_begin();v_it!=id_mesh->mesh.vertices_end();v_it++)
    {
        id_mesh->mesh.point(v_it.handle()).data()[0]=tmpmesh.point(vv_it.handle()).data()[0];
        id_mesh->mesh.point(v_it.handle()).data()[1]=tmpmesh.point(vv_it.handle()).data()[1];
        id_mesh->mesh.point(v_it.handle()).data()[2]=tmpmesh.point(vv_it.handle()).data()[2];
        vv_it++;
    }
    //id_mesh->updateGL();

   // OpenMesh::IO::write_mesh(idget->id_mesh->mesh,dir_face_database + "111.obj");
}
