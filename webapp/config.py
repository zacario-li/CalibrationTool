"""Flask Configuration for Calibration Tool"""

class BaseConfig:
    """Base configuration"""
    SECRET_KEY = 'dev-secret-key-change-in-production'
    MAX_CONTENT_LENGTH = 500 * 1024 * 1024  # 500 MB


class DevelopmentConfig(BaseConfig):
    """Development configuration"""
    DEBUG = True


class ProductionConfig(BaseConfig):
    """Production configuration"""
    DEBUG = False


config = {
    'development': DevelopmentConfig,
    'production': ProductionConfig,
    'default': DevelopmentConfig
}
