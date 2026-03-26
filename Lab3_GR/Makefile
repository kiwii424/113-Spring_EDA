CXX := g++
CXXFLAGS := -I ./inc -std=c++11
LINKFLAGS := -pedantic -fomit-frame-pointer -funroll-all-loops -O3

SRCDIR := src
OBJDIR := obj
INCDIR := inc
SRCS := $(wildcard $(SRCDIR)/*.cpp)
OBJS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))
DEPS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.d,$(SRCS))

TESTDIR := ../publicCase
OUTPUTDIR := result

.PHONY: all clean test1 test2 test3 check1 check2 check3

TARGET = D2DGRter

all: $(TARGET)

$(OBJDIR):
	@mkdir $(OBJDIR)

D2DGRter: main.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) $^ -o $@

-include $(DEPS)

#these %.o will be run when we use the OBJS, then it will find the src.cpp and use it
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR) Makefile
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -MMD -c $< -o $@

test:
	./D2DGRter $(TESTDIR)/testcase0/testcase0.gmp $(TESTDIR)/testcase0/testcase0.gcl $(TESTDIR)/testcase0/testcase0.cst $(TESTDIR)/testcase0/testcase0.lg
	./D2DGRter $(TESTDIR)/testcase1/testcase1.gmp $(TESTDIR)/testcase1/testcase1.gcl $(TESTDIR)/testcase1/testcase1.cst $(TESTDIR)/testcase1/testcase1.lg
	./D2DGRter $(TESTDIR)/testcase2/testcase2.gmp $(TESTDIR)/testcase2/testcase2.gcl $(TESTDIR)/testcase2/testcase2.cst $(TESTDIR)/testcase2/testcase2.lg
	./D2DGRter $(TESTDIR)/testcase3/testcase3.gmp $(TESTDIR)/testcase3/testcase3.gcl $(TESTDIR)/testcase3/testcase3.cst $(TESTDIR)/testcase3/testcase3.lg
	./D2DGRter $(TESTDIR)/testcase4/testcase4.gmp $(TESTDIR)/testcase4/testcase4.gcl $(TESTDIR)/testcase4/testcase4.cst $(TESTDIR)/testcase4/testcase4.lg

check:
	./Evaluator $(TESTDIR)/testcase0 testcase0
	./Evaluator $(TESTDIR)/testcase1 testcase1
	./Evaluator $(TESTDIR)/testcase2 testcase2
	./Evaluator $(TESTDIR)/testcase3 testcase3
	./Evaluator $(TESTDIR)/testcase4 testcase4

test0:
	@./D2DGRter $(TESTDIR)/testcase0/testcase0.gmp $(TESTDIR)/testcase0/testcase0.gcl $(TESTDIR)/testcase0/testcase0.cst $(TESTDIR)/testcase0/testcase0.lg
	@./Evaluator $(TESTDIR)/testcase0 testcase0
	rm -r $(TESTDIR)/testcase0/testcase0.lg
	rm -rf $(OBJDIR) $(TARGET)

test1:
	@./D2DGRter $(TESTDIR)/testcase1/testcase1.gmp $(TESTDIR)/testcase1/testcase1.gcl $(TESTDIR)/testcase1/testcase1.cst $(TESTDIR)/testcase1/testcase1.lg
	@./Evaluator $(TESTDIR)/testcase1 testcase1
	rm -r $(TESTDIR)/testcase0/testcase1.lg
	rm -rf $(OBJDIR) $(TARGET)

test2:
	@./D2DGRter $(TESTDIR)/testcase2/testcase2.gmp $(TESTDIR)/testcase2/testcase2.gcl $(TESTDIR)/testcase2/testcase2.cst $(TESTDIR)/testcase2/testcase2.lg
	@./Evaluator $(TESTDIR)/testcase2 testcase2
	rm -r $(TESTDIR)/testcase0/testcase2.lg
	rm -rf $(OBJDIR) $(TARGET)

test3:
	@./D2DGRter $(TESTDIR)/testcase3/testcase3.gmp $(TESTDIR)/testcase3/testcase3.gcl $(TESTDIR)/testcase3/testcase3.cst $(TESTDIR)/testcase3/testcase3.lg
	@./Evaluator $(TESTDIR)/testcase3 testcase3
	rm -r $(TESTDIR)/testcase0/testcase3.lg
	rm -rf $(OBJDIR) $(TARGET)

test4:
	@./D2DGRter $(TESTDIR)/testcase4/testcase4.gmp $(TESTDIR)/testcase4/testcase4.gcl $(TESTDIR)/testcase4/testcase4.cst $(TESTDIR)/testcase4/testcase4.lg
	@./Evaluator $(TESTDIR)/testcase4 testcase4
	rm -r $(TESTDIR)/testcase0/testcase4.lg
	rm -rf $(OBJDIR) $(TARGET)


$(OUTPUTDIR):
	@mkdir $(OUTPUTDIR)


clean:
	rm -rf $(OBJDIR) $(TARGET) ./110511210 110511210.tar
	rm -r $(TESTDIR)/testcase0/testcase0.lg
	rm -r $(TESTDIR)/testcase1/testcase1.lg
	rm -r $(TESTDIR)/testcase2/testcase2.lg
	rm -r $(TESTDIR)/testcase3/testcase3.lg
	rm -r $(TESTDIR)/testcase4/testcase4.lg

