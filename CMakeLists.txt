idf_build_get_property(project_dir PROJECT_DIR)

# Initialize an empty list to hold conditional requirements
set(conditionally_required_components "")

# Check if the component is present in the project not needed for this drv_i2c - should always persist
# if(EXISTS "${project_dir}/components/drv_i2c")
#     list(APPEND conditionally_required_components "drv_i2c")
# endif()

idf_component_register(
                    SRCS 
                        "drv_hdc1080.c" 
                    INCLUDE_DIRS 
                        "." 
                    REQUIRES 
                        "drv_i2c"
                        ${conditionally_required_components}
                                      )
                 

