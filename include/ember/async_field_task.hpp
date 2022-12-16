// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/logging/progress_logger.hpp>
#include <frantic/volumetrics/field_interface.hpp>

#include <boost/chrono/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/move/move.hpp>
#include <boost/thread/future.hpp>
#include <boost/thread/thread.hpp>
//#include <boost/atomic.hpp>

namespace ember {

namespace detail {
class task_impl_interface;
}

/**
 * If we have a task that produces a field asynchronously we can use this object to synchronize access, cancellation,
 * and progress logging.
 *
 * Based on the Boost.Thread library's shared_future and packaged_task objects, we are producing a field result
 * asynchronously. While the task is running, we may still be interested in logging progress updates or cancelling the
 * calculation.
 *
 * Clients can wait for the result to be produced (blocking the calling thread), can check if a result is present, can
 * query progress, and can cancel the operation.
 */
class future_field_base : public boost::enable_shared_from_this<future_field_base> {
  public:
    typedef boost::shared_ptr<future_field_base> ptr_type;
    typedef boost::shared_ptr<frantic::volumetrics::field_interface> field_ptr_type;

  public:
    template <class Callable>
    explicit future_field_base( BOOST_RV_REF( Callable ) taskImpl );

    explicit future_field_base( const boost::shared_future<field_ptr_type>& existingFuture );

    template <class Callable>
    future_field_base( BOOST_RV_REF( Callable ) taskImpl, ptr_type antecedent );

    template <class Callable, class Iterator>
    future_field_base( BOOST_RV_REF( Callable ) taskImpl, Iterator itAntecedents, Iterator itEndAntecedents );

    ~future_field_base();

    // Blocks the calling thread until the result is available returning a shared_ptr to the result. Rethrows any
    // exception that had been stored.
    field_ptr_type get();

    // Returns true once this->get() will not block.
    bool is_ready() const;

    // Cancels the operation.
    void cancel();

    // Returns a pair of the current and total progress.
    std::pair<int, int> get_progress() const;

    // Spawns a continuation task that will execute once the task associated with 'this' completes.
    template <class Callable>
    ptr_type then( BOOST_RV_REF( Callable ) );

  private:
    void init();

    void add_descendent( future_field_base* descendent );

    void remove_descendent( future_field_base* descendent );

    bool can_start() const;

    int get_progress_current() const;

  private:
    boost::shared_future<field_ptr_type> m_future;
    boost::shared_ptr<detail::task_impl_interface> m_pImpl;

    std::vector<ptr_type> m_antecedents; // Tasks prior to this one. We need these in order to know when we can start
                                         // and to get the input field ptrs.
    std::list<future_field_base*> m_descendents; // Tasks after this one. We need this list to implement cancellation.

    int m_progressTotal;
};

namespace detail {
typedef future_field_base::field_ptr_type field_ptr_type;
typedef future_field_base::ptr_type future_field_base_ptr;

class task_impl_interface {
  public:
    virtual ~task_impl_interface() {}

    virtual void cancel() = 0;

    virtual void execute() = 0;

    virtual boost::shared_future<field_ptr_type> get_future() = 0;
};

template <class CharT>
std::basic_ostream<CharT>& print_time( std::basic_ostream<CharT>& out ) {}
} // namespace detail

inline void future_field_base::init() {
    m_progressTotal = 1;
    m_future = m_pImpl->get_future();

    for( std::vector<ptr_type>::const_iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd;
         ++it ) {
        // Link this new task as a descendent of each antecedent
        ( *it )->add_descendent( this );

        // Accumulate the number of upstream nodes for use as progress tracking
        m_progressTotal += ( *it )->m_progressTotal;
    }

    try {
        m_pImpl->execute();
    } catch( ... ) {
        // We failed to execute, therefore we must remove 'this' from the task graph since it is likely still being
        // constructed and the destructor won't run.
        // TODO: Investigate if the destructor runs after this exception is thrown.
        for( std::vector<ptr_type>::const_iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd;
             ++it )
            ( *it )->remove_descendent( this );
        m_antecedents.clear();
        throw;
    }
}

inline future_field_base::~future_field_base() {
    // Cancel the task's implementation. We don't need to wait for the cancellation to complete because the task_impl
    // will have a shared_ptr to itself somewhere (or be completed/never run). If the impl is NULL, we already cancelled
    // it.
    if( m_pImpl )
        m_pImpl->cancel();

    // If we are being destroyed we must tell the antecedent tasks which are tracking 'this'
    for( std::vector<ptr_type>::iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd; ++it )
        ( *it )->remove_descendent( this );

    // Don't need to do anything with our descendents since they must be deleted already or they would have a shared_ptr
    // to 'this'.
    assert( m_descendents.empty() );
}

inline future_field_base::field_ptr_type future_field_base::get() { return m_future.get(); }

inline bool future_field_base::is_ready() const { return m_future.is_ready(); }

inline void future_field_base::cancel() {
    m_pImpl->cancel();
    m_pImpl.reset();

    for( std::list<future_field_base*>::iterator it = m_descendents.begin(), itEnd = m_descendents.end(); it != itEnd;
         ++it )
        ( *it )->cancel();
}

// Returns a pair of the current and total progress.
inline std::pair<int, int> future_field_base::get_progress() const {
    return std::make_pair( m_progressTotal - this->get_progress_current(), m_progressTotal );
}

inline void future_field_base::add_descendent( future_field_base* descendent ) {
    m_descendents.push_back( descendent );
}

inline void future_field_base::remove_descendent( future_field_base* descendent ) {
    std::list<future_field_base*>::iterator it = std::find( m_descendents.begin(), m_descendents.end(), descendent );

    assert( it != m_descendents.end() );

    if( it != m_descendents.end() )
        m_descendents.erase( it );
}

inline bool future_field_base::can_start() const {
    // TBB would use a reference count or somesuch that becomes 0 when this task is ready to go. We are going to check
    // all our held antecedents
    for( std::vector<ptr_type>::const_iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd;
         ++it ) {
        if( !( *it )->is_ready() )
            return false;
    }
    return true;
}

inline int future_field_base::get_progress_current() const {
    if( this->is_ready() )
        return 0;

    // This is not strictly accurate since it counts shared subtrees multiple times, but its good enough for progress.
    int result = 1;
    for( std::vector<ptr_type>::const_iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd;
         ++it )
        result += ( *it )->get_progress_current();

    return result;
}

namespace detail {
class task_impl_base : public task_impl_interface, public boost::enable_shared_from_this<task_impl_base> {
  public:
    task_impl_base();

    task_impl_base( future_field_base_ptr antecedent );

    template <class Iterator>
    task_impl_base( Iterator itAntecedents, Iterator itEndAntecedents );

    virtual ~task_impl_base();

    virtual void cancel();

    virtual void execute();

    virtual boost::shared_future<field_ptr_type> get_future();

  protected:
    virtual void execute_impl() = 0;

  protected:
    std::vector<future_field_base_ptr> m_antecedents;
    boost::promise<field_ptr_type> m_promise;

    class progress_logger : public frantic::logging::progress_logger {
      public:
        virtual void set_title( const frantic::tstring& /*title*/ ) {}
        virtual void update_progress( long long /*completed*/, long long /*maximum*/ ) {
            boost::this_thread::interruption_point();
        }
        virtual void update_progress( float /*percent*/ ) { boost::this_thread::interruption_point(); }
        virtual void check_for_abort() { boost::this_thread::interruption_point(); }
    } m_progress;

  private:
    boost::thread m_implThread;
};

inline task_impl_base::task_impl_base() {}

inline task_impl_base::task_impl_base( future_field_base_ptr antecedent ) { m_antecedents.push_back( antecedent ); }

template <class Iterator>
inline task_impl_base::task_impl_base( Iterator itAntecedents, Iterator itEndAntecedents )
    : m_antecedents( itAntecedents, itEndAntecedents ) {}

inline task_impl_base::~task_impl_base() {
    // Don't need to interrupt the thread because it either: 1. Never ran, or 2. Finished since the thread would have a
    // shared_ptr to this otherwise.
    if( m_implThread.joinable() )
        m_implThread.detach();
}

inline void task_impl_base::cancel() {
    if( m_implThread.joinable() ) {
        FF_LOG( debug ) << _T("Begin cancellation of thread: ") << m_implThread.get_id() << _T(" at time: ")
                        << boost::chrono::system_clock::now() << std::endl;

        m_implThread.interrupt();
        m_implThread.detach();
    }
}

inline void task_impl_base::execute() {
    assert( !m_implThread.joinable() ); // Means we executed twice!

    boost::thread newThread( &task_impl_base::execute_impl, this->shared_from_this() );

    m_implThread = boost::move( newThread );

    FF_LOG( debug ) << _T("Task started in thread: ") << m_implThread.get_id() << _T(" at time: ")
                    << boost::chrono::system_clock::now() << std::endl;
}

inline boost::shared_future<field_ptr_type> task_impl_base::get_future() { return m_promise.get_future().share(); }

template <class Callable>
class task_impl0 : public task_impl_base {
    Callable m_fn;

  public:
    explicit task_impl0( BOOST_RV_REF( Callable ) fnImpl )
        : m_fn( boost::move( fnImpl ) )
        , task_impl_base() {}

  protected:
    virtual void execute_impl() {
        try {
            Callable localImpl =
                boost::move( m_fn ); // Move the task locally so that it is destroyed when this function ends.

            boost::this_thread::interruption_point();

            field_ptr_type pResult = localImpl( m_progress );

            boost::this_thread::interruption_point();

            m_promise.set_value( pResult );

            FF_LOG( debug ) << _T("Task completed in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;
        } catch( const boost::thread_interrupted& ) {
            FF_LOG( debug ) << _T("Cancellation noted in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;

            m_promise.set_exception( boost::copy_exception( frantic::logging::progress_cancel_exception( "" ) ) );
            throw; // Must allow thread_interrupted to propogate.
        } catch( ... ) {
            m_promise.set_exception( boost::current_exception() );
        }
    }
};

template <class Callable>
class task_impl1 : public task_impl_base {
    Callable m_fn;

  public:
    task_impl1( BOOST_RV_REF( Callable ) fnImpl, future_field_base_ptr antecedent )
        : m_fn( boost::move( fnImpl ) )
        , task_impl_base( antecedent ) {}

  protected:
    virtual void execute_impl() {
        try {
            Callable localImpl =
                boost::move( m_fn ); // Move the task locally so that it is destroyed when this function ends.

            boost::this_thread::interruption_point();

            field_ptr_type pInput = m_antecedents.front()->get();

            // Drop the antecedent references because they are all completed and can be deleted if this is the last
            // reference.
            m_antecedents.clear();

            boost::this_thread::interruption_point();

            field_ptr_type pResult = localImpl( pInput, m_progress );

            boost::this_thread::interruption_point();

            m_promise.set_value( pResult );

            FF_LOG( debug ) << _T("Task completed in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;
        } catch( const boost::thread_interrupted& ) {
            FF_LOG( debug ) << _T("Cancellation noted in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;

            m_promise.set_exception( boost::copy_exception( frantic::logging::progress_cancel_exception( "" ) ) );
            throw; // Must allow thread_interrupted to propogate.
        } catch( ... ) {
            m_promise.set_exception( boost::current_exception() );
        }
    }
};

template <class Callable>
class task_impl : public task_impl_base {
    Callable m_fn;

  public:
    template <class Iterator>
    task_impl( BOOST_RV_REF( Callable ) fnImpl, Iterator itAntecedents, Iterator itEndAntecedents )
        : m_fn( boost::move( fnImpl ) )
        , task_impl_base( itAntecedents, itEndAntecedents ) {}

  protected:
    virtual void execute_impl() {
        try {
            Callable localImpl =
                boost::move( m_fn ); // Move the task locally so that it is destroyed when this function ends.

            boost::this_thread::interruption_point();

            // Typically there should only be 0 or 1 antecedents so we could optimize for that.
            // If our implementation was backed by TBB or somesuch, we could use a counter to determine when this task
            // is ready to run. A semaphore essentially.
            std::vector<field_ptr_type> inputFields;
            for( std::vector<ptr_type>::iterator it = m_antecedents.begin(), itEnd = m_antecedents.end(); it != itEnd;
                 ++it )
                inputFields.push_back( ( *it )->get() );

            // Drop the antecedent references because they are all completed and can be deleted if this is the last
            // reference.
            m_antecedents.clear();

            boost::this_thread::interruption_point();

            field_ptr_type pResult = localImpl( inputFields, m_progress );

            boost::this_thread::interruption_point();

            m_promise.set_value( pResult );

            FF_LOG( debug ) << _T("Task completed in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;
        } catch( const boost::thread_interrupted& ) {
            FF_LOG( debug ) << _T("Cancellation noted in thread: ") << boost::this_thread::get_id() << _T(" at time: ")
                            << boost::chrono::system_clock::now() << std::endl;

            m_promise.set_exception( boost::copy_exception( frantic::logging::progress_cancel_exception( "" ) ) );
            throw; // Must allow thread_interrupted to propogate.
        } catch( ... ) {
            m_promise.set_exception( boost::current_exception() );
        }
    }
};

class existing_future_impl : public task_impl_interface {
    boost::shared_future<future_field_base::field_ptr_type> m_future;

  public:
    explicit existing_future_impl( const boost::shared_future<future_field_base::field_ptr_type>& existingFuture )
        : m_future( existingFuture ) {}

    virtual void cancel() {}

    virtual void execute() {}

    virtual boost::shared_future<field_ptr_type> get_future() { return m_future; }
};
} // namespace detail

template <class Callable>
inline future_field_base::future_field_base( BOOST_RV_REF( Callable ) taskImpl )
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
    : m_pImpl( boost::make_shared<detail::task_impl0<Callable>>( boost::move( taskImpl ) ) )
#else
    : m_pImpl( new detail::task_impl0<Callable>( boost::move( taskImpl ) ) )
#endif
{
    this->init();
}

inline future_field_base::future_field_base( const boost::shared_future<field_ptr_type>& existingFuture )
    : m_pImpl( boost::make_shared<detail::existing_future_impl>( existingFuture ) ) {
    this->init();
}

template <class Callable>
inline future_field_base::future_field_base( BOOST_RV_REF( Callable ) taskImpl, ptr_type antecedent )
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
    : m_pImpl( boost::make_shared<detail::task_impl1<Callable>>( boost::move( taskImpl ), antecedent ) )
    , m_antecedents( &antecedent, &antecedent + 1 )
#else
    : m_pImpl( new detail::task_impl1<Callable>( boost::move( taskImpl ), antecedent ) )
    , m_antecedents( &antecedent, &antecedent + 1 )
#endif
{
    this->init();
}

template <class Callable, class Iterator>
inline future_field_base::future_field_base( BOOST_RV_REF( Callable ) taskImpl, Iterator itAntecedents,
                                             Iterator itEndAntecedents )
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
    : m_pImpl(
          boost::make_shared<detail::task_impl<Callable>>( boost::move( taskImpl ), itAntecedents, itEndAntecedents ) )
    , m_antecedents( itAntecedents, itEndAntecedents )
#else
    : m_pImpl( new detail::task_impl<Callable>( boost::move( taskImpl ), itAntecedents, itEndAntecedents ) )
    , m_antecedents( itAntecedents, itEndAntecedents )
#endif
{
    this->init();
}

template <class Callable>
inline future_field_base::ptr_type future_field_base::then( BOOST_RV_REF( Callable ) taskImpl ) {
#ifndef BOOST_NO_CXX11_RVALUE_REFERENCES
    return boost::make_shared<future_field_base>( boost::move( taskImpl ), this->shared_from_this() );
#else
    return ptr_type( new future_field_base( boost::move( taskImpl ), this->shared_from_this() ) );
#endif
}

// We need a way to start the async task, returning a future_field_ptr to the caller.
template <class Callable>
inline future_field_base::ptr_type create_field_task( BOOST_RV_REF( Callable ) taskImpl ) {
    return boost::shared_ptr<future_field_base>( new future_field_base( boost::move( taskImpl ) ) );
}

// Creates an async task that starts after an arbitrary number of other tasks complete. Use future_field_base::then() if
// you only have 1 antecedent.
template <class Callable>
inline future_field_base::ptr_type
create_field_task_after( BOOST_RV_REF( Callable ) taskImpl,
                         const std::vector<future_field_base::ptr_type>& antecedents ) {
    return boost::make_shared<future_field_base>( boost::move( taskImpl ), antecedents.begin(), antecedents.end() );
}

inline future_field_base::ptr_type create_field_task( future_field_base::field_ptr_type pExistingField ) {
    const boost::shared_future<future_field_base::field_ptr_type> future = boost::make_shared_future( pExistingField );

    return boost::make_shared<future_field_base>( future );
}

inline future_field_base::ptr_type
create_field_task_from_future( const boost::shared_future<future_field_base::field_ptr_type>& pExternalFutureField ) {
    return boost::make_shared<future_field_base>( pExternalFutureField );
}

} // namespace ember
