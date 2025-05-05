# 컴파일러 및 옵션
CXX = g++
CXXFLAGS += -Wall -std=c++23 -g -fsanitize=address
LDFLAGS += -fsanitize=address

# 타겟 이름
TARGET = tsp_parser

# 소스 및 오브젝트
SRCS = conv2list.cpp pugixml.cpp
OBJS = $(SRCS:.cpp=.o)

# 기본 빌드 명령
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

# 일반 정리 명령
clean:
	@echo "🧹 Cleaning all object files and core dumps..."
	@rm -f *.o *.out core

# cpp, h, hpp, tsv, xml 제외하고 모두 삭제
coreclean:
	@echo "🧼 Removing all files except .cpp, .h, .hpp, .tsp, and .xml..."
	@find . -type f ! -name '*.cpp' ! -name '*.h' ! -name '*.hpp' \
		! -name '*.tsp' ! -name '*.xml' ! -name '*Makefile' -exec rm -v {} \;

.PHONY: all clean coreclean
