#include "anno.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

//	//设置中文字体  
//	a.setFont(QFont("Microsoft Yahei", 9));
//
//	//设置中文编码
//#if (QT_VERSION <= QT_VERSION_CHECK(5,0,0))
//#if _MSC_VER
//	QTextCodec *codec = QTextCodec::codecForName("gbk");
//#else
//	QTextCodec *codec = QTextCodec::codecForName("utf-8");
//#endif
//	QTextCodec::setCodecForLocale(codec);
//	QTextCodec::setCodecForCStrings(codec);
//	QTextCodec::setCodecForTr(codec);
//#else
//	QTextCodec *codec = QTextCodec::codecForName("utf-8");
//	QTextCodec::setCodecForLocale(codec);
//#endif

	a.setWindowIcon(QIcon(":/icon/sli.ico"));

	//QSettings *appConfigFile = new QSettings("./config.ini", QSettings::IniFormat);
	//std::string curLanguage = appConfigFile->value("form/language").toString().toLatin1().toStdString();

	QSettings *reg = new QSettings("HKEY_CURRENT_USER\\SOFTWARE\\VisenPick", QSettings::NativeFormat);
	std::string curLanguage = reg->value("Language").toString().toLatin1().toStdString();

	QTranslator * AppTranslator(new QTranslator);
	AppTranslator->load(":/qmFile/pickingdemo_zh.qm");

	/*a.installTranslator(AppTranslator);
	a.removeTranslator(AppTranslator);*/
	if (curLanguage == "chinese") {
		qApp->installTranslator(AppTranslator);
	}


	QFile file("./log.txt");
	if (!file.open(QIODevice::Append | QIODevice::Text)) {
		
	}

	QTextStream out(&file);
	out << "argc: " << argc << '\n';
	for (auto i = 0; i < argc; ++i) {
		std::string argvString(argv[i]);
		out << "argv: " << QString::fromStdString(argvString) << '\n';
	}
	file.close();

	pickingDemo w;

	//if (argc == 2) {
	//	w.setProjectFilePath(argv[1]);
	//	w.OnInitFromProjectFileClick();
	//}
	
	w.setWindowTitle("anno");
    w.show();
    return a.exec();
}
