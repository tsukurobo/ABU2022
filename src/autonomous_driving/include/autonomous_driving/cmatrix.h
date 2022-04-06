#ifndef CMATRIX_H
#define CMATRIX_H

#include <iostream>
#include <string>
#include "math.h"

namespace custom_math
{
	class Matrix
	{
	private:
		double *data_ = NULL;
		static Matrix *tmp_;
		bool usable_ = false;
		int nrow_ = 0;
		int ncol_ = 0;
		static int max_nrow_;
		static int max_ncol_;

	public:
		struct MatrixCalcException
		{
		private:
			std::string str;

		public:
			MatrixCalcException(const char *s)
			: str(s)
			{}
			
			std::string what()
			{
				return str;
			}
		};

		Matrix();  //デフォルトコンストラクタ
		Matrix(int, int);  //サイズを指定して行列を作成
		Matrix(const Matrix &);  //コピーコンストラクタ
		~Matrix();  //デストラクタ
		/*各種演算子*/
		Matrix operator + (const Matrix &) const;
		Matrix operator - (const Matrix &) const;
		Matrix operator * (const Matrix &) const;
		Matrix operator * (float) const;
		Matrix &operator = (const Matrix &);  //代入演算子
		Matrix &operator += (const Matrix &);  //複合代入演算子1
		Matrix &operator *= (float);  //複合代入演算子2
		double &operator () (int, int);  //指定した要素の値を返す
		const double &operator () (int, int) const;  //()のconst対応版
		int getRowSize() const;  //行数を返す
		int getColSize() const;  //列数を返す
		/*各種操作関数など*/
		Matrix T() const;  //転置した行列を返す
		void switchRows(int, int);  //列を入れ替える
		static Matrix identity(int);  //n次単位行列を返す
		static Matrix zero(int, int);  //零行列を返す
		static Matrix rotation2x2(double);  //2次の回転行列を返す
		static void allocTmpSpace(int, int);  //行列演算で用いる一時領域を確保する
		static void freeTmpSpace();  //確保した一時領域を開放する
	};
	
	/*色々計算用の関数*/
	bool isZero(double x)
	{
		return (-1.0e-12 < x) && (x < 1.0e-12);
	}
	bool solvSimulEqs(const Matrix &, const Matrix &, Matrix &); //ガウスの消去法を使って連立方程式を解く
	Matrix calcEigenVals2x2(const Matrix &);
	Matrix calcInverse2x2(const Matrix &);  //正則行列でなければ、エラーを吐く
	void showMatrix(Matrix &m)
	{
		for(int i=0; i<m.getRowSize(); i++)
		{
			for(int j=0; j<m.getColSize(); j++)
			{
				std::cout << m(i, j) << " ";
			}
			std::cout << std::endl;
		}
		//std::cout << std::endl;
	}

	//////////*Matrixクラスの各メンバ関数の実装*//////////
	Matrix *Matrix::tmp_ = NULL;
	int Matrix::max_nrow_ = 0;
	int Matrix::max_ncol_ = 0;

 	Matrix::Matrix()
	/*: data_(NULL), usable_(false), nrow_(0), ncol_(0)*/
	{} 

	Matrix::Matrix(int nr, int nc)
	: usable_(true), nrow_(nr), ncol_(nc)
	{
		//std::cout << "constructor is called" << std::endl;
		data_ = new double[nrow_*ncol_];

		//要素の初期化
		for(int i=0; i<nrow_*ncol_; i++)
		{
			data_[i] = 0;
		}
	}

	Matrix::Matrix(const Matrix &m)
	{
		//メモリ領域がすでに確保されている場合は、そのメモリ領域にデータを移す
		//そうでない場合は、新たにメモリ領域を確保してデータを移す
		if(usable_)
		{
			//2つの行列の行数・列数が異なればエラー
			if((nrow_ != m.nrow_) || (ncol_ != m.ncol_))
			{
				MatrixCalcException ex("copy constructor: Two matrices do not have the same form.");
				throw ex;
			}

			for(int i=0; i<nrow_*ncol_; i++)
			{
				data_[i] = m.data_[i];
			}
		}
		else
		{
			data_ = new double[m.nrow_*m.ncol_];
			usable_ = true;
			nrow_ = m.nrow_;
			ncol_ = m.ncol_;

			for(int i=0; i<nrow_*ncol_; i++)
			{
				data_[i] = m.data_[i];
			}
		}
		//std::cout << "copy constructor is called!" << std::endl;
	}

	Matrix::~Matrix()
	{
		//std::cout << "destructor is called" << std::endl;
		if(usable_)
		{
			delete[] data_;
		}
	}

	double &Matrix::operator () (const int index_r, const int index_c)
	{
		//配列の要素外参照が起こったらエラー
		if(0 > (index_r*ncol_ + index_c) || (index_r*ncol_ + index_c) >= nrow_*ncol_)
		{
			MatrixCalcException ex("(): The index is out of range.");
			throw ex;
		}

		return data_[index_r*ncol_ + index_c];
	}

 	const double &Matrix::operator () (int index_r, int index_c) const
	{
		if(0 > (index_r*ncol_ + index_c) || (index_r*ncol_ + index_c) >= nrow_*ncol_)
		{
			//配列の要素外参照が起こったらエラー
			MatrixCalcException ex("(): The index is out of range.");
			throw ex;
		}

		return data_[index_r*ncol_ + index_c];
	}
 
	
	// +, -, *演算子は、一時領域に結果を格納して戻り値として返す。
	Matrix Matrix::operator + (const Matrix &m) const
	{
		//2つの行列の行数・列数が異なればエラー
		if((nrow_ != m.nrow_) || (nrow_ != m.nrow_))
		{
			MatrixCalcException ex("+: Two matrices do not have the same form.");
			throw ex;
		}

		if(m.nrow_*m.ncol_ > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("+: Tmporary matrix will overflow.");
			throw ex;
		}

		//Matrix new_mat(nrow_, ncol_);
		tmp_->nrow_ = nrow_;
		tmp_->ncol_ = ncol_;
		for (int i = 0; i < ncol_*nrow_; i++)
		{
			//new_mat.data_[i] = data_[i] + m.data_[i];
			tmp_->data_[i] = data_[i] + m.data_[i];
		}

		//return new_mat;
		return *tmp_;
	}

	Matrix Matrix::operator - (const Matrix &m) const
	{
		//2つの行列の行数・列数が異なればエラー
		if((nrow_ != m.nrow_) || (nrow_ != m.nrow_))
		{
			MatrixCalcException ex("-: Two matrices do not have the same form.");
			throw ex;
		}

		if(m.nrow_*m.ncol_ > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("-: Tmporary matrix will overflow.");
			throw ex;
		}

		//Matrix new_mat(nrow_, ncol_);
		tmp_->nrow_ = nrow_;
		tmp_->ncol_ = ncol_;
		for (int i = 0; i < nrow_*ncol_; i++)
		{
			//new_mat.data_[i] = data_[i] - m.data_[i];
			tmp_->data_[i] = data_[i] - m.data_[i];
		}

		//return new_mat;
		return *tmp_;
	}

	Matrix Matrix::operator * (const Matrix &m) const
	{
		//行列の積が定義できなければエラー
		if(ncol_ != m.nrow_)
		{
			MatrixCalcException ex("*(matrix): Can not perform multiplification.");
			throw ex;
		}

		if(nrow_*m.ncol_ > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("*(matrix): Tmporary matrix will overflow.");
			throw ex;
		}

		int i, j, k;
		double x;
		//Matrix new_mat(nrow_, m.ncol_);
		tmp_->nrow_ = nrow_;
		tmp_->ncol_ = m.ncol_;
		for (i = 0; i < nrow_; i++) {
			for (j = 0; j < m.ncol_; j++) {
				x = 0;
				for (k = 0; k < ncol_; k++) {
					x += data_[i*ncol_ + k]*m.data_[k*m.ncol_ + j];
				}
				//new_mat(i, j) = x;
				tmp_->data_[i*m.ncol_ + j] = x;
			}
		}

		//return new_mat;
		return *tmp_;
	}

	Matrix Matrix::operator * (float x) const
	{
		//初期化されていない行列に対しては、エラーを出す
		if(data_ == NULL)
		{
			MatrixCalcException ex("*(float): Operation to an uninitialized matrix is invalid.");
			throw ex;
		}

		if(nrow_*ncol_ > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("*(float): Tmporary matrix will overflow.");
			throw ex;
		}

		//Matrix new_mat(nrow_, ncol_);
		tmp_->nrow_ = nrow_;
		tmp_->ncol_ = ncol_;
		for (int i = 0; i < nrow_*ncol_; i++)
		{
			//new_mat.data_[i] = data_[i] * x;
			tmp_->data_[i] = data_[i] * x;
		}

		//return new_mat;
		return *tmp_;
	}

	Matrix &Matrix::operator = (const Matrix &m)
	{
		//メモリ領域がすでに確保されている場合は、そのメモリ領域にデータを移す
		//そうでない場合は、新たにメモリ領域を確保してデータを移す
		if(usable_)
		{
			if(nrow_ != m.nrow_ || ncol_ != m.ncol_)
			{
				//メモリ領域がすでに確保されている場合、2つの行列の行数・列数が異なればエラー
				MatrixCalcException ex("=: Two matrices do not have the same form.");
				throw ex;
			}

			for(int i=0; i<nrow_*ncol_; i++)
			{
				data_[i] = m.data_[i];
			}
		}
		else
		{
			data_ = new double[m.nrow_*m.ncol_];
			usable_ = true;
			nrow_ = m.nrow_;
			ncol_ = m.ncol_;

			for(int i=0; i<nrow_*ncol_; i++)
			{
				data_[i] = m.data_[i];
			}
		}
		//std::cout << "copy constructor is called!" << std::endl;
		return *this;
	}

	//複合代入演算子は、使用時に新しいメモリ領域を確保することはない
	Matrix &Matrix::operator += (const Matrix &m)
	{
		for (int i = 0; i < nrow_*ncol_; i++)
		{
			data_[i] += m.data_[i];
		}

		return *this;
	}

	Matrix &Matrix::operator *= (float x)
	{
		for (int i = 0; i < nrow_*ncol_; i++)
		{
			data_[i] *= x;
		}

		return *this;
	}

	//これも一時領域を使用する
	Matrix Matrix::T() const
	{
		//Matrix transposed(ncol_, nrow_);
		if(nrow_*ncol_ > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("T(): Tmporary matrix will overflow.");
			throw ex;
		}

		tmp_->nrow_ = ncol_;
		tmp_->ncol_ = nrow_;
		// for(int i=0; i<transposed.nrow_; i++)
		// {
		// 	for(int j=0; j<transposed.ncol_; j++)
		// 	{
		// 		transposed(i, j) = (*this)(j, i);
		// 	}
		// }
		for(int i=0; i<tmp_->nrow_; i++)
		{
			for(int j=0; j<tmp_->ncol_; j++)
			{
				//transposed(i, j) = (*this)(j, i);
				tmp_->data_[i*tmp_->ncol_ + j] = data_[j*ncol_ + i];
			}
		}

		//return transposed;
		return *tmp_;
	}

	void Matrix::switchRows(int row1, int row2)
	{
		double tmp;

		for (int i = 0; i < ncol_; i++){
			tmp = data_[row1*ncol_ + i];
			data_[row1*ncol_ + i] = data_[row2*ncol_ + i];
			data_[row2*ncol_ + i] = tmp;
		}
	}

	int Matrix::getRowSize() const
	{
		return nrow_;
	}

	int Matrix::getColSize() const
	{
		return ncol_;
	}

	Matrix Matrix::identity(int n)
	{
		if(n*n > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("T(): Tmporary matrix will overflow.");
			throw ex;
		}

		//Matrix tmp(n, n);
		tmp_->nrow_ = n;
		tmp_->ncol_ = n;
		for(int i=0; i<n; i++)
		{
			for(int j=0; j<n; j++)
			{
				if(i == j) tmp_->data_[i*n+j] = 1;
				else tmp_->data_[i*n+j] = 0;
			}
			
		}

		return *tmp_;
	}

	Matrix Matrix::zero(int nr, int nc)
	{
		if(nr*nc > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("T(): Tmporary matrix will overflow.");
			throw ex;
		}
		
		tmp_->nrow_ = nr;
		tmp_->ncol_ = nc;
		for(int i=0; i<nr*nc; i++)
		{
			tmp_->data_[i] = 0;
		}

		return *tmp_;
	}

	Matrix Matrix::rotation2x2(double theta)
	{
		if(4 > max_nrow_*max_ncol_)
		{
			MatrixCalcException ex("T(): Tmporary matrix will overflow.");
			throw ex;
		}
		//Matrix tmp(2, 2);
		tmp_->nrow_ = 2;
		tmp_->ncol_ = 2;
		tmp_->data_[0] = cos(theta); tmp_->data_[1] = -sin(theta);
		tmp_->data_[2] = sin(theta); tmp_->data_[3] = cos(theta);

		return *tmp_;
	}

	void Matrix::allocTmpSpace(int max_row, int max_col)
	{
		tmp_ = new Matrix(max_row, max_col);
		max_nrow_ = max_row;
		max_ncol_ = max_col;
	}

	void Matrix::freeTmpSpace()
	{
		delete tmp_;
	}

	//A→係数行列(正方行列)　b→右辺ベクトル c→解ベクトル
	//A, b, cはすべて適切に初期化されていることを前提とする(手抜き仕様)
	bool solvSimulEqs(const Matrix &A, const Matrix &b, Matrix &c)
	{
		bool solvable = true;

		//ガウスの消去法v2
		Matrix tmpA = A; //一時保管用行列の作成
		Matrix tmpb = b;
		double x, y = 0;

		//前進消去ステップ
		int i, j, k;
		for (i = 0; i < tmpA.getRowSize(); i++) {
			//ピボット0が来ないようにする
			if (isZero(tmpA(i, i)))
			{
				if(i == tmpA.getRowSize()-1)
				{
					solvable = false;
				}
				else
				{
					//std::cout << "loop..." << std::endl;
					for (k = i + 1; isZero(tmpA(k, i)); k++)
					{
						if(k == tmpA.getRowSize()-1)
						{
							solvable = false;
							break;
						}
					}
				}
				
				if(solvable == false) return false;
				
				tmpA.switchRows(i, k);
				tmpb.switchRows(i, k);
			}

			for (j = i + 1; j < tmpA.getRowSize(); j++) {
				x = tmpA(j, i) / tmpA(i, i);
				for (k = i; k < tmpA.getColSize(); k++) {
					tmpA(j, k) = tmpA(j, k) - tmpA(i, k)*x;
				}
				tmpb(j, 0) = tmpb(j, 0) - tmpb(i, 0)*x;
			}
		}

		//後退代入ステップ
		for (i = c.getRowSize()-1; i >= 0; i--) {
			y = 0;
			for (j = i+1; j < tmpA.getColSize(); j++) {
				y += tmpA(i, j)*c(j, 0);
			}
			c(i, 0) = (tmpb(i, 0) - y) / tmpA(i, i);
		}

		return true;
	}

	Matrix calcEigenVals2x2(const Matrix &m)
	{
		if(m.getRowSize() != 2 || m.getColSize() != 2)
		{
			Matrix::MatrixCalcException ex("The matrix passed has inappropriate form.");
			throw ex;
		}

		Matrix tmp(2, 1);

		tmp(0, 0) = 0.5*(
			m(0, 0)+m(1, 1) + 
			sqrt( 
				(m(0, 0)+m(1, 1))*(m(0, 0)+m(1, 1)) - 
				4.0*(m(0, 0)*m(1, 1)-m(0, 1)*m(1, 0))
				)
			);
		tmp(1, 0) = 0.5*(
			m(0, 0)+m(1, 1) - 
			sqrt( 
				(m(0, 0)+m(1, 1))*(m(0, 0)+m(1, 1)) - 
				4.0*(m(0, 0)*m(1, 1)-m(0, 1)*m(1, 0))
				)
			);

		return tmp;
	}

	Matrix calcInverse2x2(const Matrix &m)
	{
		if(m.getRowSize() != 2 || m.getColSize() != 2)
		{
			Matrix::MatrixCalcException ex("The matrix passed has inappropriate form.");
			throw ex;
		}

		Matrix tmp(2, 2);
		double det_m = m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0);
		if(isZero(det_m))
		{
			Matrix::MatrixCalcException ex("The matrix passed does not have a inverse matrix.");
			throw ex;
		}
		double inv_det_m = 1.0/det_m;

		tmp(0, 0) = m(1, 1)*inv_det_m;  tmp(0, 1) = -m(0, 1)*inv_det_m;
		tmp(1, 0) = -m(1, 0)*inv_det_m; tmp(1, 1) = m(0, 0)*inv_det_m;

		return tmp; 
	}
}

#endif
