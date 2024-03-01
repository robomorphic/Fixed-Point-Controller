#ifndef __fpc__util_hpp__
#define __fpc__util_hpp__
#include "config.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

#define PRINT_VECTOR(x, of) of << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { of << std::setprecision(8) << x[i] << " "; } of << std::endl;
//#define PRINT_VECTOR(x) std::cout << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { std::cout << x[i] << " "; } std::cout << std::endl;
//#define PRINT_VECTOR_VECTOR(x) std::cout << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { for(int j = 0; j < x[i].size(); j++) { std::cout << x[i][j] << " "; } std::cout << std::endl; }
#define PRINT_VECTOR_VECTOR(x, of) of << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { for(int j = 0; j < x[i].size(); j++) { of << std::setprecision(8) << x[i][j] << " "; } of << std::endl; }

template<typename T>
void print_model_helper(
    pinocchio::ModelTpl<T> model,
    std::ofstream &model_file
) {
    model_file << "nq: " << model.nq << std::endl;
    model_file << "nv: " << model.nv << std::endl;
    model_file << "nbodies: " << model.nbodies << std::endl;
    model_file << "nframes: " << model.nframes << std::endl;
    PRINT_VECTOR(model.parents, model_file);
    PRINT_VECTOR(model.names, model_file);
    PRINT_VECTOR_VECTOR(model.supports, model_file);
    PRINT_VECTOR_VECTOR(model.subtrees, model_file);
    model_file << "gravity: " << model.gravity << std::endl;
    model_file << "name: " << model.name << std::endl;

    PRINT_VECTOR(model.idx_qs, model_file);
    PRINT_VECTOR(model.nqs, model_file);
    PRINT_VECTOR(model.idx_vs, model_file);
    PRINT_VECTOR(model.nvs, model_file);

    PRINT_VECTOR(model.rotorInertia, model_file);
    PRINT_VECTOR(model.rotorGearRatio, model_file);
    PRINT_VECTOR(model.friction, model_file);
    PRINT_VECTOR(model.damping, model_file);
    PRINT_VECTOR(model.effortLimit, model_file);
    PRINT_VECTOR(model.velocityLimit, model_file);
    PRINT_VECTOR(model.lowerPositionLimit, model_file);
    PRINT_VECTOR(model.upperPositionLimit, model_file);

    // print reference configurations
    for(auto const& conf: model.referenceConfigurations) {
        std::cout << conf.first << ": ";
        PRINT_VECTOR(conf.second, model_file);
    }
    
    PRINT_VECTOR(model.inertias, model_file);
    PRINT_VECTOR(model.jointPlacements, model_file);
    PRINT_VECTOR(model.joints, model_file);
    PRINT_VECTOR(model.frames, model_file);
};

/*
 * This function will print every variable in the model
 * The output is "model_output_filename" + model_original.txt
 * model_output_filename + model_modified.txt
*/
template <typename T, typename T2>
void print_model(
    pinocchio::ModelTpl<T> original_model,
    pinocchio::ModelTpl<T2> modified_model
) {
    // open the file
    std::ofstream model_original_file;
    std::ofstream model_modified_file;

    // mkdir the folder if needed
    std::filesystem::create_directories(model_output_foldername);
    model_original_file.open(model_output_foldername + "model_original.txt");
    model_modified_file.open(model_output_foldername + "model_modified.txt");

    // print the original model
    print_model_helper(original_model, model_original_file);
    print_model_helper(modified_model, model_modified_file);
}

#endif