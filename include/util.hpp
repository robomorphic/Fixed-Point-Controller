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

template <typename NewScalar>
void model_cast(
    pinocchio::ModelTpl<double> &model,
    pinocchio::ModelTpl<NewScalar> &new_model
) {
    new_model = model.cast<NewScalar>();
    new_model.nq = NewScalar(model.nq);
    new_model.nv = NewScalar(model.nv);
    new_model.njoints = NewScalar(model.njoints);
    new_model.nbodies = NewScalar(model.nbodies);
    new_model.nframes = NewScalar(model.nframes);
    // clear parents, names, supports, subtrees
    new_model.parents.clear();
    new_model.names.clear();
    new_model.supports.clear();
    new_model.subtrees.clear();
    for(int i = 0; i < model.parents.size(); i++) {
        new_model.parents.push_back(model.parents[i]);
    }
    for(int i = 0; i < model.names.size(); i++) {
        new_model.names.push_back(model.names[i]);
    }
    for(int i = 0; i < model.supports.size(); i++) {
        new_model.supports.push_back(model.supports[i]);
    }
    for(int i = 0; i < model.subtrees.size(); i++) {
        new_model.subtrees.push_back(model.subtrees[i]);
    }
    new_model.gravity = model.gravity.template cast<NewScalar>();
    new_model.name = model.name;

    // clear these vectors
    new_model.idx_qs.clear();
    new_model.nqs.clear();
    new_model.idx_vs.clear();
    new_model.nvs.clear();
    for(int i = 0; i < model.idx_qs.size(); i++) {
        new_model.idx_qs.push_back(NewScalar(model.idx_qs[i]));
    }
    for(int i = 0; i < model.nqs.size(); i++) {
        new_model.nqs.push_back(NewScalar(model.nqs[i]));
    }
    for(int i = 0; i < model.idx_vs.size(); i++) {
        new_model.idx_vs.push_back(NewScalar(model.idx_vs[i]));
    }
    for(int i = 0; i < model.nvs.size(); i++) {
        new_model.nvs.push_back(NewScalar(model.nvs[i]));
    }

    // clear vectors
    std::cerr << "BE CAREFUL, in some systems casting may give wrong results" << std::endl;
    std::cerr << "gravity: " << model.gravity << std::endl;
    std::cerr << "new gravity: " << new_model.gravity << std::endl;

    // Eigen Vectors
    new_model.rotorInertia          = model.rotorInertia.template cast<NewScalar>();
    new_model.rotorGearRatio        = model.rotorGearRatio.template cast<NewScalar>();
    new_model.friction              = model.friction.template cast<NewScalar>();
    new_model.damping               = model.damping.template cast<NewScalar>();
    new_model.effortLimit           = model.effortLimit.template cast<NewScalar>();
    new_model.velocityLimit         = model.velocityLimit.template cast<NewScalar>();
    new_model.lowerPositionLimit    = model.lowerPositionLimit.template cast<NewScalar>();
    new_model.upperPositionLimit    = model.upperPositionLimit.template cast<NewScalar>();

    // I'll see if this is necessary
    //typename ConfigVectorMap::const_iterator it;
    //for (it = model.referenceConfigurations.begin();
    //    it != model.referenceConfigurations.end(); it++ )
    //{
    //new_model.referenceConfigurations.insert(std::make_pair(it->first, it->second.template cast<NewScalar>()));
    //}

    // reserve vectors
    new_model.inertias.resize(model.inertias.size());
    new_model.jointPlacements.resize(model.jointPlacements.size());
    new_model.joints.resize(model.joints.size());
    new_model.frames.resize(model.frames.size());

    /// copy into vectors
    for(size_t k = 0; k < model.joints.size(); ++k)
    {
        new_model.inertias[k] = model.inertias[k].template cast<NewScalar>();
        new_model.jointPlacements[k] = model.jointPlacements[k].template cast<NewScalar>();
        new_model.joints[k] = model.joints[k].template cast<NewScalar>();
    }
    // the values in the for loop may be wrong, now rewrite NewScalars
    for(size_t k = 0; k < model.frames.size(); ++k)
    {
        new_model.inertias[k] = InertiaTpl<NewScalar>(model.inertias[k].mass(), model.inertias[k].lever().template cast<NewScalar>(), model.inertias[k].inertia().template cast<NewScalar>());
    }
    
    for(size_t k = 0; k < model.frames.size(); ++k)
    {
    new_model.frames[k] = model.frames[k].template cast<NewScalar>();
    }

}



#endif