#include <QApplication>
#include "lobovoxelizor.h"

int main(int argc, char *argv[])
{
    // Show the max long integer available in the system.
    cout << "Max int = " << std::numeric_limits<int>::max() << endl;
    cout << "Max long = " << std::numeric_limits<long>::max() << endl;
    cout << "Max long long = " << std::numeric_limits<long long>::max() << endl;

	QApplication a(argc, argv);

    LoboVoxelizor w;
    w.show();

    return a.exec();
}
