OBJ_CUNIT = $(BUILD_FOLDER)/nrf_cunit.o $(BUILD_FOLDER)/nrf_cunit_mock.o $(BUILD_FOLDER)/cunit_test.o

CUNIT_INC = -I$(TRUNK_ROOT)/framework/cunit/include_override
CUNIT_INC += -I$(TRUNK_ROOT)/framework/cunit
CUNIT_INC += -I$(TRUNK_ROOT)/Nordic/nrf51/Include/gcc
CUNIT_INC += -I$(TRUNK_ROOT)/Nordic/nrf51/Include

MOCK_GPIOTE = $(BUILD_FOLDER)/app_gpiote_mock.o

MOCK_UART= $(BUILD_FOLDER)/app_uart_mock.o

MOCK_TIMER = $(BUILD_FOLDER)/app_timer_mock.o


$(MOCK_GPIOTE): $(TRUNK_ROOT)/Nordic/nrf51/Source/app_common/mock/app_gpiote/app_gpiote_mock.c
	$(CC) -c $< -o $@ $(CFLAGS)
	
$(MOCK_UART): $(TRUNK_ROOT)/Nordic/nrf51/Source/app_common/mock/app_uart_mock.c
	$(CC) -c $< -o $@ $(CFLAGS)
	
$(BUILD_FOLDER)/nrf_cunit.o: $(TRUNK_ROOT)/framework/cunit/nrf_cunit.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BUILD_FOLDER)/nrf_cunit_mock.o: $(TRUNK_ROOT)/framework/cunit/nrf_cunit_mock.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BUILD_FOLDER)/cunit_test.o: cunit_test.c
	$(CC) -c $< -o $@ $(CFLAGS)
    
cunit_test: $(OBJ)
	$(CPP) $(OBJ) -o $(BUILD_FOLDER)/$(BIN) $(LIBS)
    
.PHONY: test test_dump
test:
	./$(BUILD_FOLDER)/$(BIN)

test_dump:
	"./$(BUILD_FOLDER)/$(BIN)" > "./$(BUILD_FOLDER)/cunit_test.log"
