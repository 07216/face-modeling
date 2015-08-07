#ifndef CONFIGPARAMEER_H
#define CONFIGPARAMEER_H
#include <string>
namespace ConfigParameter {
    const static std::string rootDir = "Data/";
    const static std::string choseIndexDir = rootDir + "ChoseIndex/";
    const static std::string templateDir = rootDir + "Template/";
    const static std::string avatarModelsDir = rootDir + "avatar-models/";
    const static std::string facedatabaseDir = rootDir + "face_database/";
    const static std::string TStarDir = rootDir + "T-Star-new/";
    const static std::string gl_EDir = templateDir + "gl_E/";

    const static std::string config_filename = rootDir + "Config.ini";
    const static std::string face_recognition_selection_index = choseIndexDir + "face_recognition_selection_index.txt";
    const static std::string mesh_face_filename = rootDir + "face.txt";

    //lbf
    const static std::string lbfDir = rootDir + "LBF_Tracking_Model/";
    const static std::string lbf_keypoint_index_filename = lbfDir + "key_point_lbf.txt";
    const static std::string mesh_keypoint_index_filename = lbfDir + "key_point_mesh.txt";
    const static std::string lbf_tracker_filename = lbfDir + "Track_51.bin";
    const static std::string lbf_detector_filename = lbfDir + "FaceRecognitionModel.bin";

    //all register
    const static std::string template_mesh_filename = templateDir + "template.obj";
    const static std::string neutral_meanface_filename = templateDir + "neutral_meanface.txt";
    const static std::string neutral_eigenvalue_filename = templateDir + "neutral_eigenvalue.txt";
    const static std::string neutral_eigenvector_filename = templateDir + "neutral_eigenvector.txt";
    const static std::string neutral_gl_eigenvalue_filename = templateDir + "gl_eigenvalue.txt";
    const static std::string neutral_gl_eigenvector_filename = templateDir + "gl_eigenvector.txt";

    //init register
    const static std::string init_register_index_filenname = choseIndexDir + "nonrigid_selection_index.txt";
    const static std::string jaw_register_index_filename = choseIndexDir + "jaw_selection_index.txt";
    const static std::string fixed_register_index_filename = choseIndexDir + "mouth_nose_eye_selection_index.txt";

    //rigid register
    const static std::string rigid_register_index_filename = choseIndexDir + "rigid_selection_index.txt";
    const static std::string nose_register_index_filename = choseIndexDir + "nose_region_selection_index.txt";

    //nonrigid register
    const static std::string nonrigid_register_index_filename = choseIndexDir + "nonrigid_selection_index.txt";
    const static std::string update_register_index_filename = choseIndexDir + "update_selection_index.txt";
    const static std::string mouth_brow_upsample_index_filename = choseIndexDir + "mouth_brow_upsample_selection_index.txt";
    const static std::string optical_index_filename = choseIndexDir + "mouth_brow_jaw_optical_index.txt";

}
#endif
