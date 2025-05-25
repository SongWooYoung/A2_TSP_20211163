# 컴파일러 및 옵션
CXX = g++
CXXFLAGS += -Wall -std=c++23 -g -fsanitize=address
LDFLAGS += -fsanitize=address

# 타겟 목록
TARGETS = tsp_parser christofides test test2 test3

# 타겟별 소스 및 오브젝트 파일
SRCS_tsp_parser = conv2list.cpp pugixml.cpp
SRCS_christofides = christofides.cpp conv2list.cpp pugixml.cpp
SRCS_test = test.cpp conv2list.cpp pugixml.cpp Held_Karp.cpp
SRCS_test2 = test2.cpp Held_Karp.cpp
SRCS_test3 = test3.cpp Held_Karp.cpp christofides.cpp mcpm2.cpp approx2.cpp #mcpm.cpp

OBJS_tsp_parser = $(SRCS_tsp_parser:.cpp=.o)
OBJS_christofides = $(SRCS_christofides:.cpp=.o)
OBJS_test = $(SRCS_test:.cpp=.o)
OBJS_test2 = $(SRCS_test2:.cpp=.o)
OBJS_test3 = $(SRCS_test3:.cpp=.o)

# 기본 타겟 (모두 빌드)
all: $(TARGETS)

tsp_parser: $(OBJS_tsp_parser)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

christofides: $(OBJS_christofides)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test: $(OBJS_test)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test2: $(OBJS_test2)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

test3: $(OBJS_test3)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)


%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	@echo "🧹 Cleaning all object files and executables..."
	@rm -f *.o *.out core $(TARGETS)

coreclean:
	@echo "🧼 Removing all files except .cpp, .h, .hpp, .tsp, and .xml..."
	@find . -type f ! -name '*.cpp' ! -name '*.h' ! -name '*.hpp' \
		! -name '*.tsp' ! -name '*.xml' ! -name '*Makefile' -exec rm -v {} \;

.PHONY: all clean coreclean
