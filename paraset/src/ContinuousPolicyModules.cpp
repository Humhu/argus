#include "paraset/ContinuousPolicyModules.h"
#include "argus_utils/utils/MatrixUtils.h"

#define POSDEF_OFFSET_SCALE (1E-9)
#define RELU_LEAKY_SLOPE (1E-3)

using namespace percepto;

namespace argus
{

ContinuousPolicyModule::ContinuousPolicyModule() {}

ContinuousPolicyModule::~ContinuousPolicyModule() {}

std::ostream& operator<<( std::ostream& os, const ContinuousPolicyModule& m )
{
	os << m.Print();
	return os;
}

LinearGaussian::LinearGaussian( unsigned int inputDim,
                                unsigned int matDim )
: mean( inputDim, matDim ),
  correlations( matDim*(matDim-1)/2 ),
  logVariances( matDim )
{
	variances.SetSource( &logVariances );
	psdModule.SetLSource( &correlations );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
	information.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

LinearGaussian::LinearGaussian( const LinearGaussian& other )
: mean( other.mean ),
  correlations( other.correlations ),
  logVariances( other.logVariances ),
  information( other.information )
{
	variances.SetSource( &logVariances );
	psdModule.SetLSource( &correlations );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
}

void LinearGaussian::SetInputSource( VectorSourceType* src )
{
	mean.SetSource( src );
}

LinearGaussian::VectorSourceType&
LinearGaussian::GetMeanSource()
{
	return mean;
}

LinearGaussian::MatrixSourceType&
LinearGaussian::GetInfoSource()
{
	return information;
}

void LinearGaussian::Foreprop()
{
	correlations.Foreprop();
	logVariances.Foreprop();
}

void LinearGaussian::Invalidate()
{
	correlations.Invalidate();
	logVariances.Invalidate();
}

percepto::Parameters::Ptr LinearGaussian::CreateMeanParameters()
{
	return mean.CreateParameters();
}

percepto::Parameters::Ptr LinearGaussian::CreateCovParameters()
{
	Parameters::Ptr corrParams = correlations.CreateParameters();
	Parameters::Ptr logVarParams = logVariances.CreateParameters();
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( corrParams );
	params->AddParameters( logVarParams );
	return params;
}

void LinearGaussian::InitializeMean( const VectorType& u )
{
	mean.SetOffsets( u );
}

void LinearGaussian::InitializeInformation( const MatrixType& n )
{
	Eigen::LDLT<MatrixType> ldlt( n );
	VectorType d = ldlt.vectorD();
	VectorType l = GetLowerTriangular( MatrixType( ldlt.matrixL() ), 1 );
	logVariances.SetOutput( d.array().log().matrix() );
	correlations.SetOutput( l );
}

std::string LinearGaussian::Print() const
{
	std::stringstream ss;
	ss << "Mean regressor: " << std::endl << mean.GetWeights() << std::endl;
	// TODO: Display L and D terms
	return ss.str();
}

FixedVarianceGaussian::FixedVarianceGaussian( unsigned int inputDim, 
                                              unsigned int matDim,
                                              unsigned int numHiddenLayers,
                                              unsigned int layerWidth )
: mean( inputDim, 
        matDim, 
        numHiddenLayers, 
        layerWidth, 
        percepto::SigmoidActivation(),
        percepto::PerceptronNet::OUTPUT_RECTIFIED ),
  correlations( matDim*(matDim-1)/2 ),
  logVariances( matDim )
{
	variances.SetSource( &logVariances );
	psdModule.SetLSource( &correlations );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
	information.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
FixedVarianceGaussian::FixedVarianceGaussian( const FixedVarianceGaussian& other )
: mean( other.mean ),
  correlations( other.correlations ),
  logVariances( other.logVariances ),
  variances( other.variances ),
  psdModule( other.psdModule ),
  information( other.information ),
  corrParams( other.corrParams )
{
	variances.SetSource( &logVariances );
	psdModule.SetLSource( &correlations );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
}

void FixedVarianceGaussian::SetInputSource( VectorSourceType* src )
{
	mean.SetSource( src );
}

FixedVarianceGaussian::VectorSourceType&
FixedVarianceGaussian::GetMeanSource()
{
	return mean.GetOutputSource();
}

FixedVarianceGaussian::MatrixSourceType&
FixedVarianceGaussian::GetInfoSource()
{
	return information;
}

void FixedVarianceGaussian::Foreprop()
{
	correlations.Foreprop();
	logVariances.Foreprop();
}

void FixedVarianceGaussian::Invalidate()
{
	correlations.Invalidate();
	logVariances.Invalidate();
}

percepto::Parameters::Ptr FixedVarianceGaussian::CreateMeanParameters()
{
	return mean.CreateParameters();
}

percepto::Parameters::Ptr FixedVarianceGaussian::CreateCovParameters()
{
	corrParams = correlations.CreateParameters();
	Parameters::Ptr logVarParams = logVariances.CreateParameters();
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( corrParams );
	params->AddParameters( logVarParams );
	return params;
}

void FixedVarianceGaussian::InitializeMean( const VectorType& u )
{
	mean.SetOutputOffsets( u );
}

void FixedVarianceGaussian::InitializeInformation( const MatrixType& n )
{
	Eigen::LDLT<MatrixType> ldlt( n );
	VectorType d = ldlt.vectorD();
	VectorType l = GetLowerTriangular( MatrixType( ldlt.matrixL() ), 1 );
	logVariances.SetOutput( d.array().log().matrix() );
	correlations.SetOutput( l );
}

std::string FixedVarianceGaussian::Print() const
{
	std::stringstream ss;
	ss << "Mean regressor: " << std::endl << mean << std::endl;
	// TODO: Display L and D terms
	return ss.str();
}

VariableVarianceGaussian::VariableVarianceGaussian( unsigned int inputDim, 
                                                    unsigned int matDim,
                                                    unsigned int numHiddenLayers,
                                                    unsigned int layerWidth )
: mean( inputDim, 
        matDim, 
        numHiddenLayers, 
        layerWidth, 
        percepto::SigmoidActivation(), 
        percepto::PerceptronNet::OUTPUT_RECTIFIED ),
  correlations( inputDim, 
                matDim*(matDim-1)/2, 
                numHiddenLayers, 
                layerWidth,
                percepto::SigmoidActivation(), 
                percepto::PerceptronNet::OUTPUT_UNRECTIFIED ),
  logVariances( inputDim, 
                matDim, 
                numHiddenLayers, 
                layerWidth,
                percepto::SigmoidActivation(), 
                percepto::PerceptronNet::OUTPUT_UNRECTIFIED )
{
	variances.SetSource( &logVariances.GetOutputSource() );
	psdModule.SetLSource( &correlations.GetOutputSource() );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
	information.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
VariableVarianceGaussian::VariableVarianceGaussian( const VariableVarianceGaussian& other )
: mean( other.mean ),
  correlations( other.correlations ),
  logVariances( other.logVariances ),
  variances( other.variances ),
  information( other.information ),
  corrParams( other.corrParams )
{
	variances.SetSource( &logVariances.GetOutputSource() );
	psdModule.SetLSource( &correlations.GetOutputSource() );
	psdModule.SetDSource( &variances );
	information.SetSource( &psdModule );
}

void VariableVarianceGaussian::SetInputSource( VectorSourceType* src )
{
	mean.SetSource( src );
	correlations.SetSource( src );
	logVariances.SetSource( src );
}

VariableVarianceGaussian::VectorSourceType&
VariableVarianceGaussian::GetMeanSource()
{
	return mean.GetOutputSource();
}

VariableVarianceGaussian::MatrixSourceType&
VariableVarianceGaussian::GetInfoSource()
{
	return information;
}

void VariableVarianceGaussian::InitializeMean( const VectorType& u )
{
	mean.SetOutputOffsets( u );
}

void VariableVarianceGaussian::InitializeInformation( const MatrixType& n )
{
	Eigen::LDLT<MatrixType> ldlt( n );
	VectorType d = ldlt.vectorD();
	VectorType l = GetLowerTriangular( MatrixType( ldlt.matrixL() ), 1 );
	logVariances.SetOutputOffsets( d.array().log().matrix() );
	correlations.SetOutputOffsets( l );
}

percepto::Parameters::Ptr VariableVarianceGaussian::CreateMeanParameters()
{
	return mean.CreateParameters();
}

percepto::Parameters::Ptr VariableVarianceGaussian::CreateCovParameters()
{
	corrParams = correlations.CreateParameters();
	percepto:: Parameters::Ptr logVarParams = logVariances.CreateParameters();
	
	VectorType w = corrParams->GetParamsVec();
	w.setZero();
	corrParams->SetParamsVec( w );
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	// params->AddParameters( corrParams );
	params->AddParameters( logVarParams );
	return params;
}

void VariableVarianceGaussian::Foreprop()
{}

void VariableVarianceGaussian::Invalidate()
{}

std::string VariableVarianceGaussian::Print() const
{
	std::stringstream ss;
	ss << "Mean regressor: " << std::endl << mean << std::endl;
	ss << "Var regressor: " << std::endl << logVariances << std::endl;
	// ss << "Corr regressor: " << std::endl << correlations << std::endl;
	return ss.str();
}

}