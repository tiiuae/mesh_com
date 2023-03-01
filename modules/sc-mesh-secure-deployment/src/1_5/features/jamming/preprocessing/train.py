import lightgbm as lgb
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.style
import numpy as np
import pandas as pd
import xgboost as xgb
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import f1_score
from sklearn.metrics import plot_confusion_matrix
from sklearn.model_selection import RandomizedSearchCV
from sklearn.model_selection import cross_val_score
from sklearn.svm import SVC

mpl.style.use('classic')


def train_rf(train_features, train_labels, binary=True):
    scoring = 'f1' if binary else 'accuracy'
    classifier = RandomForestClassifier()
    parameters = {
        'bootstrap': [True, False],
        'max_depth': [5, 10, 20, 30, 40, 50, None],
        'max_features': ['sqrt', 'log2', None],
        'min_samples_leaf': [1, 2, 4],
        'min_samples_split': [2, 4, 8],
        'n_estimators': [50, 100, 150, 200]
    }
    rs_model = RandomizedSearchCV(
        classifier, param_distributions=parameters, n_iter=50, cv=3, scoring=scoring, n_jobs=3, verbose=0
    )
    rs_model.fit(train_features, train_labels)
    best_estimator = rs_model.best_estimator_
    print(rs_model.best_params_)
    return best_estimator


def train_svm(train_features, train_labels, binary=True):
    scoring = 'f1' if binary else 'accuracy'
    classifier = SVC()
    parameters = {
        'kernel': ['rbf'],  # ['rbf', 'poly'],
        'C': [0.1, 1, 5, 10, 15, 20, 25, 30, 35],
        'gamma': ['scale', 'auto', 0.0001, 0.001, 0.01, 0.1, 1.0],
        # 'degree': [2, 3, 4, 5, 6]
    }
    rs_model = RandomizedSearchCV(
        classifier, param_distributions=parameters, n_iter=50, cv=3, scoring=scoring, n_jobs=5, verbose=0
    )
    rs_model.fit(train_features, train_labels)
    best_estimator = rs_model.best_estimator_
    print(rs_model.best_params_)
    return best_estimator


def train_lgbm(train_features, train_labels, binary=True):
    scoring = 'f1' if binary else 'accuracy'
    classifier = lgb.LGBMClassifier()
    parameters = {
        'learning_rate': [0.001, 0.01, 0.05, 0.1, 0.15, 0.20, 0.25, 0.30],
        'max_depth': [5, 10, 20, 30, 40, 50, None],
        'n_estimators': [50, 100, 150, 200],
        'num_leaves': [6, 8, 12, 16],
        "min_child_weight": [1, 3, 5, 7],
        'colsample_bytree': [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0],
        'subsample': [0.2, 0.4, 0.5, 0.6, 0.7, 0.8],
        'reg_alpha': [0, 0.5, 1, 1.2],
        'reg_lambda': [1, 1.2, 1.4, 1.5, 2, 3, 4],
        'boosting_type': ['gbdt']
        # 'objective': ['multiclass'],
    }
    rs_model = RandomizedSearchCV(
        classifier, param_distributions=parameters, n_iter=50, scoring=scoring, n_jobs=5, cv=3, verbose=0
    )
    rs_model.fit(train_features, train_labels)
    best_estimator = rs_model.best_estimator_
    return best_estimator


def train_xgboost(train_features, train_labels, binary=True):
    scoring = 'f1' if binary else 'accuracy'
    classifier = xgb.XGBClassifier()
    parameters = {
        "learning_rate": [0.001, 0.01, 0.05, 0.1, 0.15, 0.20, 0.25, 0.30],
        "gamma": [0.0, 0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 1, 1.5, 2],
        'max_depth': [5, 10, 20, 30, 40, 50, None],
        "colsample_bytree": [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0],
        "subsample": [0.2, 0.4, 0.5, 0.6, 0.7],
        "reg_alpha": [0, 0.5, 1],
        "reg_lambda": [1, 1.5, 2, 3, 4.5],
        "min_child_weight": [1, 3, 5, 7],
        'n_estimators': [50, 100, 150, 200]
    }

    rs_model = RandomizedSearchCV(
        classifier, param_distributions=parameters, n_iter=50, scoring=scoring, n_jobs=5, cv=3, verbose=0
    )
    rs_model.fit(train_features, train_labels)
    best_estimator = rs_model.best_estimator_
    return best_estimator


def test(estimator, test_features, test_labels, binary=True):
    scoring = 'f1' if binary else 'accuracy'

    predictions = estimator.predict(test_features)
    if scoring == 'f1':
        accuracy = accuracy_score(test_labels, predictions)
        f1 = f1_score(test_labels, predictions)
        print('accuracy', accuracy)
        print('f1', f1)
    else:
        accuracy = accuracy_score(test_labels, predictions)
        print('accuracy', accuracy)

    print(confusion_matrix(test_labels, predictions))
    cm = plot_confusion_matrix(estimator, test_features, test_labels)
    fig_name = 'binary_cm' if binary else 'multiclass_cm'
    plt.tight_layout()
    cm.figure_.savefig(f'results/{estimator.__class__.__name__}_{fig_name}.png', dpi=300)
    plt.close()

    fig_name = 'binary_fi' if binary else 'multiclass_fi'
    if type(estimator) == xgb.XGBClassifier:
        ax = xgb.plot_importance(estimator, height=0.35)
        fig = ax.figure
        fig.set_size_inches(10, 10)
        plt.tight_layout()
        ax.figure.savefig(f'results/{estimator.__class__.__name__}_{fig_name}.png', dpi=300)
    elif type(estimator) == lgb.LGBMClassifier:
        ax = lgb.plot_importance(estimator, height=0.35)
        fig = ax.figure
        fig.set_size_inches(10, 10)
        plt.tight_layout()
        ax.figure.savefig(f'results/{estimator.__class__.__name__}_{fig_name}.png', dpi=300)
    elif type(estimator) == RandomForestClassifier:
        feat_importances = pd.Series(estimator.feature_importances_, index=test_features.columns)
        ax = feat_importances.nlargest(56).plot(kind='barh')
        ax.invert_yaxis()
        plt.tight_layout()
        ax.figure.savefig(f'results/{estimator.__class__.__name__}_{fig_name}.png', dpi=300)


def compute_time(estimator, test_features, test_labels, binary=True):
    start = time.time()
    for _ in range(10):
        predictions = estimator.predict(test_features)
    time = time.time() - start
    print('time taken', time) 


def main():
    np.random.seed(0)
    train_df = pd.read_csv('train_test_split/train.csv')
    train_bin_labels = np.array(train_df['bin_label'])
    train_labels = np.array(train_df['label'])

    test_df = pd.read_csv('train_test_split/test.csv')
    test_bin_labels = np.array(test_df['bin_label'])
    test_labels = np.array(test_df['label'])

    # Drop some features
    drop_features = ['bin_label', 'label']
    drop_keyword = ['mad']
    for c in train_df.columns:
        for keyword in drop_keyword:
            if keyword in c:
                drop_features.append(c)
    print('dropping', drop_keyword)
    train_features = train_df.drop(drop_features, axis=1)
    test_features = test_df.drop(drop_features, axis=1)

    # Define train routine
    routines = [train_svm, train_rf, train_lgbm, train_xgboost]
    for train_func in routines:
        # # Binary
        estimator = train_func(train_features, train_bin_labels, binary=True)
        test(estimator, test_features, test_bin_labels, binary=True)
        score = cross_val_score(estimator, test_features, test_bin_labels, cv=100)
        print('cv binary ', np.array(score).mean(), np.array(score).std())

        # Multi-class
        estimator = train_func(train_features, train_labels, binary=False)
        test(estimator, test_features, test_labels, binary=False)
        score = cross_val_score(estimator, test_features, test_labels, cv=100)
        print('cv multiclass', np.array(score).mean(), np.array(score).std())


if __name__ == '__main__':
    main()
