FROM vdecaro/teaching:latest
WORKDIR /code
RUN python3 -m poetry install --sync --only main -E sensors
ENV PATH="/code/.venv/bin:${PATH}"
WORKDIR /app
ENV TEACHING_PACKAGE=sensors
CMD ["python3", "run.py"]



# FROM sensors_stage AS video_stage
# ARG ARCH
# RUN apt-get -y update -qq --fix-missing && \
#     apt-get -y install --no-install-recommends \
#     unzip \
#     cmake \
#     ffmpeg \
#     libtbb2 \
#     gfortran \
#     apt-utils \
#     pkg-config \
#     checkinstall \
#     qt5-default \
#     build-essential \
#     libopenblas-base \
#     libopenblas-dev \
#     liblapack-dev \
#     libatlas-base-dev \
#     #libgtk-3-dev \
#     #libavcodec58 \
#     libavcodec-dev \
#     #libavformat58 \
#     libavformat-dev \
#     libavutil-dev \
#     #libswscale5 \
#     libswscale-dev \
#     libjpeg8-dev \
#     libpng-dev \
#     libtiff5-dev \
#     #libdc1394-22 \
#     libdc1394-22-dev \
#     libxine2-dev \
#     libv4l-dev \
#     libgstreamer1.0 \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-0 \
#     libgstreamer-plugins-base1.0-dev \
#     libglew-dev \
#     libpostproc-dev \
#     libeigen3-dev \
#     libtbb-dev \
#     zlib1g-dev \
#     libsm6 \
#     libxext6 \
#     libxrender1 \
#     git

# RUN if [ "${ARCH}" = "arm64" ]; then\
#         apt-get -y install --no-install-recommends gcc-aarch64-linux-gnu \
#         g++-aarch64-linux-gnu;\
#     fi
# RUN python3 -m pip install numpy

# RUN git clone https://github.com/opencv/opencv.git && \
#     mkdir ./opencv/build && \
#     cd ./opencv/build && \
# #
#     if [ "${ARCH}" = "amd64" ]; then \
#         cmake \
#             -D CMAKE_BUILD_TYPE=RELEASE \
#             -D BUILD_PYTHON_SUPPORT=ON \
#             -D BUILD_DOCS=ON \
#             -D BUILD_PERF_TESTS=OFF \
#             -D BUILD_TESTS=OFF \
#             -D CMAKE_INSTALL_PREFIX=/usr/local \
#             -D BUILD_opencv_python3=ON\
#             -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \      
#             -D BUILD_EXAMPLES=OFF \
#             -D WITH_IPP=OFF \
#             -D WITH_FFMPEG=ON \
#             -D WITH_GSTREAMER=ON \
#             -D WITH_V4L=ON \
#             -D WITH_LIBV4L=ON \
#             -D WITH_TBB=ON \
#             -D WITH_QT=ON \
#             -D WITH_OPENGL=ON \
#             -D WITH_LAPACK=ON \
#             #-D WITH_HPX=ON \
#             -D ENABLE_PRECOMPILED_HEADERS=OFF \
#             ..; \
# #
#     elif [ "${ARCH}" = "arm" ]; then \
#         cmake \
#             -D CMAKE_BUILD_TYPE=RELEASE \
#             -D BUILD_PYTHON_SUPPORT=ON \
#             -D ENABLE_NEON=ON \
#             -D ENABLE_VFPV3=OFF \
#             -D WITH_OPENMP=OFF \
#             -D WITH_OPENCL=OFF \
#             -D BUILD_ZLIB=ON \
#             -D BUILD_TIFF=ON \
#             -D WITH_FFMPEG=ON \
#             -D WITH_TBB=OFF \
#             -D BUILD_TBB=OFF \
#             -D BUILD_TESTS=OFF \
#             -D WITH_EIGEN=OFF \
#             -D WITH_GSTREAMER=ON \
#             -D WITH_V4L=ON \
#             -D WITH_LIBV4L=ON \
#             -D WITH_VTK=OFF \
#             -D WITH_QT=OFF \
#             -D BUILD_DOCS=ON \
#             -D BUILD_PERF_TESTS=OFF \
#             -D BUILD_opencv_python3=ON \
#             -D CMAKE_INSTALL_PREFIX=/usr/local \
#             -D BUILD_EXAMPLES=OFF \
#             -D ENABLE_PRECOMPILED_HEADERS=OFF \
#             -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
#             ..; \
#     fi && \
# #
# #   Build, Test and Install
#     make -j 4 && \
#     make install && \
#     ldconfig && \
# #
# #   Cleaning
#     apt-get -y remove \
#         unzip \
#         cmake \
#         gfortran \
#         apt-utils \
#         pkg-config \
#         checkinstall \
#         build-essential \
#         libopenblas-dev \
#         liblapack-dev \
#         libatlas-base-dev \
#         #libgtk-3-dev \
#         libavcodec-dev \
#         libavformat-dev \
#         libavutil-dev \
#         libswscale-dev \
#         libjpeg8-dev \
#         libpng-dev \
#         libtiff5-dev \
#         libdc1394-22-dev \
#         libxine2-dev \
#         libv4l-dev \
#         libgstreamer1.0-dev \
#         libgstreamer-plugins-base1.0-dev \
#         libglew-dev \
#         libpostproc-dev \
#         libeigen3-dev \
#         libtbb-dev \
#         zlib1g-dev \
#     && \
#     apt-get autoremove -y && \
#     apt-get clean

# COPY /video /app/video
# CMD ["python3", "main.py"]
