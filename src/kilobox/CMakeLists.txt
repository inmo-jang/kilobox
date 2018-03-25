

# Some flags for Freeglut and GLUI.
add_definitions( -DFREEGLUT_EXPORTS -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

# Define the framework files.
set(Testbed_Framework_SRCS
	../Box2D/Testbed/Framework/Main.cpp
	../Box2D/Testbed/Framework/Render.cpp
	../Box2D/Testbed/Framework/Render.h
	../Box2D/Testbed/Framework/Test.cpp
	../Box2D/Testbed/Framework/Test.h
	../Box2D/Testbed/Tests/TestEntries.cpp
)

#define the test files.
set(kilobox_SRC
	kilolib.h
	kilolib.cpp
	kiloworld.h
	kiloworld.cpp
    worldfile.cpp
    worldfile.h
	evokilo1.h
	evokilo1.cpp
)


include_directories (
	${OPENGL_INCLUDE_DIR}
	../Box2D
    ../..
)

if(APPLE)
	# We are not using the Apple's framework version, but X11's
	include_directories( /usr/X11/include )
	link_directories( /usr/X11/lib )
    message(Apple)
	#set (OPENGL_LIBRARIES GL GLU GLUT X11)
	set (OPENGL_LIBRARIES "-framework OpenGL -framework GLUT")
else(APPLE)
    set (ADDITIONAL_LIBRARIES freeglut_static X11)
endif(APPLE)

add_executable(kilobox
	${Testbed_Framework_SRCS}
	${kilobox_SRC}
)
target_link_libraries (
	kilobox
	Box2D
	#freeglut_static
	glui
	${ADDITIONAL_LIBRARIES}
	${OPENGL_LIBRARIES}
) 

