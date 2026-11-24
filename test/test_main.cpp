#include <gtest/gtest.h>
#include <QLoggingCategory>
#include <QApplication>

int main(int argc, char *argv[])
{
    #ifdef _WIN32
        SetConsoleOutputCP(65001);
    #endif

    QApplication app(argc, argv);

    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}