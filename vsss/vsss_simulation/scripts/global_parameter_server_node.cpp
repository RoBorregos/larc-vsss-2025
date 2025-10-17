#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

class global_parameter_server_node: public rclcpp::Node
{
public:
    global_parameter_server_node() : Node("global_parameter_server_node", 
                                        rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true))
    {


        // Variables de Movimiento en Robot

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.set__description("Constante Angular para la velocidad linear");
        this->declare_parameter<double>("KLinear", 0.35, param_desc);
        param_desc.set__description("Constante angular proporcional, en donde el angulo de diferencia determina que tanto rota el robot");
        this->declare_parameter<double>("KpAngular", 1.6, param_desc);
        param_desc.set__description("Constante angular integral, en donde el promedio de los errores es lo que se aporta para poder hacer un movimiento suave hacia el valor");
        this->declare_parameter<double>("KiAngular", 0.3, param_desc);


        //Variables de Movimiento en CampoVectorial
        param_desc.set__description("Constante de arco dentro de Campo Vectorial");
        this->declare_parameter<double>("Campo_DE", 0.055, param_desc);

        param_desc.set__description("Constante de separacion dentro de Campo Vectorial");
        this->declare_parameter<double>("Campo_KR", 0.12, param_desc);

        param_desc.set__description("Constante velocidad de los obstaculos para su prediccion");
        this->declare_parameter<double>("Enemigo_KO", 0.01, param_desc);

        param_desc.set__description("Constante de distancia minima de activacion de repelente hacia obstaculos");
        this->declare_parameter<double>("Campo_deltaMin", 0.03, param_desc);
        
        param_desc.set__description("Constante de UnionEntre repelente y Objetivo");
        this->declare_parameter<double>("Campo_delta__", 0.05, param_desc);


        
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<global_parameter_server_node>());
    rclcpp::shutdown();
    return 0;
}