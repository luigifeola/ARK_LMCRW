#ifndef LMCRWEXP_GLOBAL_H
#define LMCRWEXP_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(LMCRWEXP_LIBRARY)
#  define LMCRWEXPSHARED_EXPORT Q_DECL_EXPORT
#else
#  define LMCRWEXPSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // VECTORGRADIENTWITHSWAPEXP_GLOBAL_H
