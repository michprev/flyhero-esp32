//
// Created by michal on 21.12.17.
//

#include <unity.h>
#include <cstring>
#include "Logger.h"

using namespace flyhero;

TEST_CASE("Basic logger test", "[logger]")
{
    Logger &logger = Logger::Instance();

    TEST_ASSERT_TRUE(logger.Init());
    TEST_ASSERT_TRUE(logger.Erase());

    bool var = true;

    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    var = false;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));


    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)) && var);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)) && var);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)) && var);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)) && !var);
}

TEST_CASE("Array data type logger test", "[logger]")
{
    Logger &logger = Logger::Instance();

    TEST_ASSERT_TRUE(logger.Init());
    TEST_ASSERT_TRUE(logger.Erase());

    int var[4] = { 1, 2, 3, 4 };

    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    var[3] = -8;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    var[1] = 0;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));


    int expected[4] = { 1, 2, 3, 4 };

    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_INT_ARRAY(expected, var, 4);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    expected[3] = -8;
    TEST_ASSERT_EQUAL_INT_ARRAY(expected, var, 4);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    expected[1] = 0;
    TEST_ASSERT_EQUAL_INT_ARRAY(expected, var, 4);
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
}

struct test_struct
{
    char *str;
    int xy;
};

TEST_CASE("Struct data type logger test", "[logger]")
{
    Logger &logger = Logger::Instance();

    TEST_ASSERT_TRUE(logger.Init());
    TEST_ASSERT_TRUE(logger.Erase());

    test_struct var;
    var.str = strdup("Hello!");
    var.xy = -5;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));

    var.str = strdup("Test string");
    var.xy = 454578;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));

    var.str = strdup("Final test!");
    var.xy = 0;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));


    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_STRING("Hello!", var.str);
    TEST_ASSERT_EQUAL_INT(-5, var.xy);
    delete var.str;
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_STRING("Test string", var.str);
    TEST_ASSERT_EQUAL_INT(454578, var.xy);
    delete var.str;
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_STRING("Final test!", var.str);
    TEST_ASSERT_EQUAL_INT(0, var.xy);
    delete var.str;
}

class test_class
{
private:
    int x;

public:
    test_class()
    {
        this->x = INT_MIN;
    }

    void Set_X(int x)
    {
        this->x = x;
    }

    int Get_X()
    {
        return this->x;
    }
};

TEST_CASE("Class data type logger test", "[logger]")
{
    Logger &logger = Logger::Instance();

    TEST_ASSERT_TRUE(logger.Init());
    TEST_ASSERT_TRUE(logger.Erase());

    test_class var;
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));

    var.Set_X(55);
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));

    var.Set_X(-9);
    TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));


    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_INT(INT_MIN, var.Get_X());
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_INT(55, var.Get_X());
    TEST_ASSERT_TRUE(logger.Read_Next(&var, sizeof(var)));
    TEST_ASSERT_EQUAL_INT(-9, var.Get_X());
}

TEST_CASE("Run out of space test", "[logger]")
{
    Logger &logger = Logger::Instance();

    TEST_ASSERT_TRUE(logger.Init());
    TEST_ASSERT_TRUE(logger.Erase());

    char var = 'a';

    for (int i = 0; i < 8 * 1024; i++)
        TEST_ASSERT_TRUE(logger.Log_Next(&var, sizeof(var)));

    TEST_ASSERT_FALSE(logger.Log_Next(&var, sizeof(var)));
    TEST_ASSERT_FALSE(logger.Log_Next(&var, sizeof(var)));
}