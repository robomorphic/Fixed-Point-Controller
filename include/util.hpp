#ifndef __fpc__util_hpp__
#define __fpc__util_hpp__
#include "config.hpp"
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

#define PRINT_VECTOR(x, of) std::cout << #x << ":"; for(int i = 0; i < x.size(); i++) { of << std::setprecision(4) << x[i] << " "; } of << std::endl;
//#define PRINT_VECTOR(x) std::cout << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { std::cout << x[i] << " "; } std::cout << std::endl;
//#define PRINT_VECTOR_VECTOR(x) std::cout << "Vector: " << #x << std::endl; for(int i = 0; i < x.size(); i++) { for(int j = 0; j < x[i].size(); j++) { std::cout << x[i][j] << " "; } std::cout << std::endl; }
#define PRINT_VECTOR_VECTOR(x, of) for(int i = 0; i < x.size(); i++) { for(int j = 0; j < x[i].size(); j++) { of << std::setprecision(4) << x[i][j] << " "; } of << std::endl; }
#define PRINT_MATRIX(x, of) for(int i = 0; i < x.rows(); i++) { for(int j = 0; j < x.cols(); j++) { of << std::setprecision(4) << x(i, j) << " "; } of << std::endl; }

template<typename T>
void ABA_output_helper(
    pinocchio::DataTpl<T> data,
    std::ofstream &file
){
    PRINT_MATRIX(data.Minv, file);
}

template<typename T, typename T2>
void print_ABA_output(
    pinocchio::DataTpl<T> original_data,
    pinocchio::DataTpl<T2> fixed_point_data
) {
    std::ofstream original_data_file;
    std::ofstream fixed_point_data_file;

    auto dir = model_output_foldername + "/ABA/" + std::to_string(CONTROLLER_ABA_PRINT_INDEX) + "/";
    std::filesystem::create_directories(dir);
    original_data_file.open(dir + "original_data.txt");
    fixed_point_data_file.open(dir + "/fixed_point_data.txt");

    ABA_output_helper(original_data, original_data_file);
    ABA_output_helper(fixed_point_data, fixed_point_data_file);
    CONTROLLER_ABA_PRINT_INDEX++;

}



template<typename T>
void print_model_helper(
    pinocchio::ModelTpl<T> model,
    std::ofstream &model_file
) {
    model_file << "integer_bits_standard: " << INT_BITS_STANDARD << std::endl;
    model_file << "fractional_bits_standard: " << FRAC_BITS_STANDARD << std::endl;
    model_file << "integer_bits_gravity: " << INT_BITS_GRAVITY << std::endl;
    model_file << "fractional_bits_gravity: " << FRAC_BITS_GRAVITY << std::endl;
    model_file << "integer_bits_fd: " << INT_BITS_FD << std::endl;
    model_file << "fractional_bits_fd: " << FRAC_BITS_FD << std::endl; 
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
        model_file << conf.first << ": ";
        PRINT_VECTOR(conf.second, model_file);
    }
    
    model_file << "model.inertias:" << std::endl;
    //PRINT_VECTOR(model.inertias, model_file);
    for(int i = 0; i < model.inertias.size(); i++) {
        model_file << "inertia: " << i << std::endl;
        model_file << model.inertias[i] << std::endl;
        model_file << model.inertias[i].matrix() << std::endl;
    }
    model_file << "model.jointPlacements:" << std::endl;
    PRINT_VECTOR(model.jointPlacements, model_file);
    PRINT_VECTOR(model.joints, model_file);
    PRINT_VECTOR(model.frames, model_file);
};

/*
 * This function will print every variable in the model
 * The output is "model_output_filename" + model_original.txt
 * model_output_filename + model_modified.txt
*/
template <typename T, typename T2, typename T3>
void print_model(
    pinocchio::ModelTpl<T> original_model,
    pinocchio::ModelTpl<T2> bigger_model,
    pinocchio::ModelTpl<T3> smaller_model
) {
    // open the file
    std::ofstream model_original_file;
    std::ofstream model_bigger_file;
    std::ofstream model_smaller_file;

    // mkdir the folder if needed
    std::filesystem::create_directories(model_output_foldername);
    model_original_file.open(model_output_foldername + "model_original.txt");
    model_bigger_file.open(model_output_foldername + "model_bigger.txt");
    model_smaller_file.open(model_output_foldername + "model_smaller.txt");

    // print the original model
    print_model_helper(original_model, model_original_file);
    print_model_helper(bigger_model, model_bigger_file);
    print_model_helper(smaller_model, model_smaller_file);
}

#endif