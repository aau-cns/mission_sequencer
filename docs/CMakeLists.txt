
#
# Target 'docs'
#

if(NOT OPTION_BUILD_DOCS)
    message(WARNING "Doc generation is turned off")
    return()
endif()

add_custom_target(docs)


#
# Documentation
#

add_subdirectory(api-docs)
