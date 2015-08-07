#ifndef ICP_H
#define ICP_H
#include <Eigen/Dense>
#include <map>
#include <vector>
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {
    enum Function {
        PNORM,
        TUKEY,
        FAIR,
        LOGISTIC,
        TRIMMED,
        LESS,
        MEDIAN,
        NONE
    };
    class Parameters {
    public:
        Parameters();
        /// Parameters
        Function f;     /// robust function type
        double p;       /// paramter of the robust function
        int max_icp;    /// max ICP iteration
        int max_outer;  /// max outer iteration
        double stop;    /// stopping criteria
    };

    void less_weight(Eigen::VectorXd& r, double p);

    /// Weight functions
    /// @param Residuals
    /// @param Parameter
    void uniform_weight(Eigen::VectorXd& r);
    /// @param Residuals
    /// @param Parameter
    void pnorm_weight(Eigen::VectorXd& r, double p, double reg=1e-8);
    /// @param Residuals
    /// @param Parameter
    void tukey_weight(Eigen::VectorXd& r, double p);
    /// @param Residuals
    /// @param Parameter
    void fair_weight(Eigen::VectorXd& r, double p);
    /// @param Residuals
    /// @param Parameter
    void logistic_weight(Eigen::VectorXd& r, double p);
    /// @param Residuals
    /// @param Parameter
    void median_weight(Eigen::VectorXd& r, double p);
    struct sort_pred {
        bool operator()(const std::pair<int,double> &left,
                        const std::pair<int,double> &right);
    };
    /// @param Residuals
    /// @param Parameter
    void trimmed_weight(Eigen::VectorXd& r, double p);
    /// @param Function type
    /// @param Residuals
    /// @param Parameter
    void robust_weight(Function f, Eigen::VectorXd& r, double p);
}


#endif // ICP_H
