#include "dev/storage/dtuide/dtu_platform.hh"

DtuPlatform::DtuPlatform(const DtuPlatformParams * p)
    : Platform(p), idedtuproxy(p->idedtuproxy)
{
}

DtuPlatform*
DtuPlatformParams::create()
{
    return new DtuPlatform(this);
}

DtuPlatform::~DtuPlatform()
{
}

void
DtuPlatform::postConsoleInt()
{
    idedtuproxy->postConsoleInt();
}

void
DtuPlatform::clearConsoleInt()
{
    idedtuproxy->clearConsoleInt();
    //maybe let idedtuproxy inherit from platform?
    //not possible because nameclash
}

void
DtuPlatform::postPciInt(int line)
{
    idedtuproxy->postPciInt(line);
}

void
DtuPlatform::clearPciInt(int line)
{
    idedtuproxy->clearPciInt(line);
}
